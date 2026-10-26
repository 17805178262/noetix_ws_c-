#pragma once
#include "frame_queue.hpp"
#include "gmr_mink.hpp"
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <optional>
#include <functional>
#include <chrono>
#include <iostream>

namespace gmr {

struct ProcessedFrame {
    double          frame_time = 0.0;
    Eigen::VectorXd qpos;
    BodyMap         body_data;
};

class MotionBuffer {
public:
    using PreprocessFn = std::function<void(BodyMap&, double)>;
    using ErrorFn = std::function<void(const std::string&)>;

    explicit MotionBuffer(size_t max_frames,
                          double frame_timeout_sec = 0.02)
        : max_frames_(max_frames)
        , frame_timeout_(frame_timeout_sec)
    {}

    ~MotionBuffer() { stopAsync(); }

    void setPreprocessFn(PreprocessFn fn) { preprocess_fn_ = std::move(fn); }
    void setErrorFn(ErrorFn fn) { error_fn_ = std::move(fn); }
    void setOffsetToGround(bool v) { offset_to_ground_ = v; }
    void setConsumeLatestFrames(bool v) { consume_latest_frames_ = v; }

    void seedSync(int n, FrameQueue& queue, gmr_mink::GMR* gmr) {
        for (int i = 0; i < n;) {
            RawFrame raw;
            if (!queue.pop(raw, frame_timeout_)) continue;
            auto pf = retarget(raw, gmr);
            if (pf) { append(*pf); ++i; }
        }
    }

    void startAsync(FrameQueue& queue, gmr_mink::GMR* gmr) {
        if (running_) return;
        running_ = true;
        thread_ = std::thread([this, &queue, gmr] {
            while (running_) {
                RawFrame raw;
                const bool got = consume_latest_frames_
                    ? queue.popLatest(raw, frame_timeout_)
                    : queue.pop(raw, frame_timeout_);
                if (!got) continue;
                auto pf = retarget(raw, gmr);
                if (pf) {
                    append(*pf);
                }
            }
        });
    }

    void stopAsync() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    void clear() {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_.clear();
        dropped_ = 0;
    }

    size_t length() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return buf_.size();
    }

    int droppedCount() const { return dropped_.load(); }

    Eigen::VectorXd latestQpos() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return {};
        return buf_.back().qpos;
    }

    BodyMap latestBodyData() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return {};
        return buf_.back().body_data;
    }

    std::pair<Eigen::VectorXd, double> latestFrame() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return {{}, 0.0};
        return {buf_.back().qpos, buf_.back().frame_time};
    }

    std::optional<ProcessedFrame> latestProcessedFrame() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return std::nullopt;
        return buf_.back();
    }

    // Sample the processed motion on a steady output clock. Pico frame times
    // use steady_clock too, so a small delay lets a faster publisher smoothly
    // interpolate between adjacent IK frames instead of holding each source
    // frame until the next callback arrives.
    std::optional<ProcessedFrame> interpolatedProcessedFrame(
        double delay_sec) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return std::nullopt;
        if (delay_sec <= 0.0 || buf_.size() < 2) return buf_.back();

        const double now_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const double target_time = now_sec - delay_sec;
        auto upper = std::lower_bound(
            buf_.begin(), buf_.end(), target_time,
            [](const ProcessedFrame& frame, double t) {
                return frame.frame_time < t;
            });
        if (upper == buf_.begin()) return *upper;
        if (upper == buf_.end()) return buf_.back();

        const auto lower = std::prev(upper);
        const double span = upper->frame_time - lower->frame_time;
        if (span <= 1e-9 || lower->qpos.size() != upper->qpos.size())
            return *upper;

        const double alpha = std::clamp(
            (target_time - lower->frame_time) / span, 0.0, 1.0);
        ProcessedFrame result;
        result.frame_time = target_time;
        result.qpos = (1.0 - alpha) * lower->qpos + alpha * upper->qpos;

        // qpos[3..6] is the free-root quaternion (wxyz); use slerp rather
        // than component-wise interpolation to keep it normalized.
        if (result.qpos.size() >= 7) {
            Eigen::Quaterniond q0(lower->qpos[3], lower->qpos[4],
                                  lower->qpos[5], lower->qpos[6]);
            Eigen::Quaterniond q1(upper->qpos[3], upper->qpos[4],
                                  upper->qpos[5], upper->qpos[6]);
            if (q0.norm() > 1e-8 && q1.norm() > 1e-8) {
                const Eigen::Quaterniond q =
                    q0.normalized().slerp(alpha, q1.normalized()).normalized();
                result.qpos[3] = q.w();
                result.qpos[4] = q.x();
                result.qpos[5] = q.y();
                result.qpos[6] = q.z();
            }
        }
        result.body_data = upper->body_data;
        return result;
    }

private:
    std::optional<ProcessedFrame> retarget(RawFrame& raw, gmr_mink::GMR* gmr) {
        try {
            if (raw.stamp_ns <= 0)
                throw std::runtime_error("Motion frame is missing a monotonic receive timestamp");
            ProcessedFrame pf;
            pf.frame_time = raw.stamp_ns * 1e-9;
            if (pf.frame_time <= last_frame_time_) return std::nullopt;
            if (preprocess_fn_) preprocess_fn_(raw.body_data, pf.frame_time);
            pf.qpos = gmr->retarget(raw.body_data, offset_to_ground_, pf.frame_time);
            if (!pf.qpos.allFinite()) throw std::runtime_error("IK output contains NaN/Inf");
            last_frame_time_ = pf.frame_time;
            pf.body_data = std::move(raw.body_data);
            return pf;
        } catch (const std::exception& e) {
            if (error_fn_) error_fn_(e.what());
            return std::nullopt;
        }
    }

    void append(ProcessedFrame pf) {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_.push_back(std::move(pf));
        if (buf_.size() > max_frames_) {
            size_t ovf = buf_.size() - max_frames_;
            buf_.erase(buf_.begin(), buf_.begin() + ovf);
            dropped_ += int(ovf);
        }
    }

    size_t              max_frames_;
    double              frame_timeout_;
    PreprocessFn        preprocess_fn_;
    ErrorFn             error_fn_;
    double              last_frame_time_ = 0;
    bool                offset_to_ground_ = false;  // default false，不影响mocap_server
    bool                consume_latest_frames_ = false;
    std::atomic<bool>   running_{false};
    std::atomic<int>    dropped_{0};
    std::thread         thread_;
    mutable std::mutex  mtx_;
    std::deque<ProcessedFrame> buf_;


};

} // namespace gmr
