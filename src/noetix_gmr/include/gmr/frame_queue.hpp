#pragma once
/**
 * frame_queue.hpp — Thread-safe raw frame queue
 *
 * Readers push RawFrames; MotionBuffer pops them.
 */

#include "body_map.hpp"
#include <cstdint>
#include <chrono>
#include <deque>
#include <mutex>
#include <condition_variable>

namespace gmr {

struct RawFrame {
    int     frame_number = 0;
    int64_t stamp_ns     = 0;
    BodyMap body_data;
};

class FrameQueue {
public:
    // 保留所有待处理帧，IK按FIFO顺序消费；不因队列积压删除旧帧。
    FrameQueue() = default;

    void push(RawFrame f) {
        std::lock_guard<std::mutex> lk(mtx_);
        q_.push_back(std::move(f));
        cv_.notify_one();
    }

    // 从队首取出下一帧，超时返回false；保持所有已入队帧的先后顺序。
    bool pop(RawFrame& out, double timeout_sec) {
        std::unique_lock<std::mutex> lk(mtx_);
        auto dl = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout_sec);
        if (!cv_.wait_until(lk, dl, [&] { return !q_.empty(); }))
            return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.size();
    }

private:
    std::deque<RawFrame>    q_;
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
};

} // namespace gmr
