#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "gmr/motion_buffer.hpp"
#include "gmr/motion_output.hpp"
#include "gmr/output_tuning.hpp"
#include "gmr/sim2real_preprocessor.hpp"
#include "readers/xsens_reader.hpp"
#ifdef GMR_WITH_OPTITRACK
#include "readers/optitrack_reader.hpp"
#endif
#ifdef GMR_WITH_FZMOTION
#include "readers/fzmotion_reader.hpp"
#endif
#ifdef GMR_WITH_PICO
#include "readers/pico_reader.hpp"
#endif
#include <filesystem>
#include <set>

namespace {
double steadySeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
class RetargetNode : public rclcpp::Node {
public:
    RetargetNode() : Node("noetix_gmr") {
        const auto share = std::filesystem::path(
            ament_index_cpp::get_package_share_directory("noetix_gmr"));
        device_ = parameter<std::string>("device", "pico");
        robot_ = parameter<std::string>("robot", "n3");
        if (device_ != "pico" && device_ != "xsens" && device_ != "optitrack" && device_ != "fzmotion")
            throw std::invalid_argument("device must be pico, xsens, optitrack or fzmotion");
        const auto preset = gmr::robotPreset(robot_);
        std::ifstream stream(share / "config/robots.json");
        nlohmann::json robots; stream >> robots;
        const auto& defaults = robots.at(robot_);
        auto xml = parameter<std::string>("model_path", "");
        auto ik = parameter<std::string>("ik_config", "");
        if (xml.empty()) xml = (share / defaults.at("model").get<std::string>()).string();
        if (ik.empty()) {
            if (!defaults.at("ik_configs").contains(device_))
                throw std::invalid_argument("No built-in IK config for " + device_ + "/" + robot_ +
                                            "; supply ik_config explicitly");
            ik = (share / defaults.at("ik_configs").at(device_).get<std::string>()).string();
        }
        const double height = positive("human_height", device_ == "pico" ? 1.6 : 1.8);
        const double damping = positive("ik_damping", device_ == "xsens" || device_ == "fzmotion" ? 1.0 : 0.5);
        const double hz = positive("publish_hz", 50.0);
        if (hz > 1000.0) throw std::invalid_argument("publish_hz must not exceed 1000");
        stale_timeout_ = positive("frame_timeout", 0.5);
        interpolation_delay_ = parameter<double>("interpolation_delay_ms", 0.0) * 0.001;
        if (!std::isfinite(interpolation_delay_) || interpolation_delay_ < 0 || interpolation_delay_ >= stale_timeout_)
            throw std::invalid_argument("interpolation delay must be nonnegative and shorter than frame_timeout");
        const bool ground = parameter<bool>("offset_to_ground", true);
        const bool lock_tilt = parameter<bool>("lock_root_tilt", false);
        const auto spine = parameter<std::string>("spine_joint", robot_ == "g1" && device_ != "pico" ? "Spine1" : "");
        const double spine_deg = finite("spine_deg", spine.empty() ? 0.0 : 6.0);
        const auto warmup = parameter<int>("warmup_iterations", 1000);
        if (warmup < 0) throw std::invalid_argument("warmup_iterations must be nonnegative");

        gmr::MotionOutput::Config output;
        output.robot = preset;
        output.pelvis_z_offset = finite("pelvis_z_offset", device_ == "optitrack" ? preset.pelvis_z_offset : 0.0);
        output.lin_vel_alpha = finite("lin_vel_alpha", 1.0);
        output.ang_vel_alpha = finite("ang_vel_alpha", 1.0);
        output.lin_vel_max = nonnegative("lin_vel_max", 0.0);
        output.ang_vel_max = nonnegative("ang_vel_max", 0.0);
        output_ = std::make_unique<gmr::MotionOutput>(output);
        tuning_ = std::make_unique<gmr::JointSpeedLimiter>(
            nonnegative("motion_scale", 1.0), finite("root_z_offset", 0.0),
            nonnegative("root_xy_scale", 1.0), nonnegative("max_joint_speed", 0.0), lock_tilt);

        solver_ = std::make_unique<gmr_mink::GMR>(xml, ik, height, damping, false);
        sim2real_profile_ = solver_->sim2realProfile();
        const bool effective_ground = sim2real_profile_ ? false : ground;
        const int effective_warmup = sim2real_profile_ ? 0 : warmup;
        auto pico_preprocessor = sim2real_profile_
            ? std::make_shared<gmr::Sim2realPicoPreprocessor>(stale_timeout_) : nullptr;
        if (solver_->model()->nq != 7 + preset.num_joints)
            throw std::invalid_argument("Model qpos size does not match robot preset");
        // Fail at startup rather than silently skipping misnamed robot targets.
        std::ifstream config_stream(ik); nlohmann::json config; config_stream >> config;
        std::set<std::string> required_bones{config.at("human_root_name").get<std::string>()};
        for (const auto& table : {"ik_match_table1", "ik_match_table2"}) {
            if (!config.value(std::string("use_") + table, false)) continue;
            for (const auto& entry : config.at(table).items()) {
                if (entry.value()[1].get<double>() == 0 && entry.value()[2].get<double>() == 0) continue;
                if (mj_name2id(solver_->model(), mjOBJ_BODY, entry.key().c_str()) < 0)
                    throw std::invalid_argument("IK target body missing from model: " + entry.key());
                required_bones.insert(entry.value()[0].get<std::string>());
            }
        }
        buffer_ = std::make_unique<gmr::MotionBuffer>(600, 0.02);
        buffer_->setOffsetToGround(effective_ground);
        buffer_->setErrorFn([this](const std::string& error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "Retarget frame rejected: %s", error.c_str());
        });
        buffer_->setPreprocessFn([this, required_bones, spine, spine_deg, effective_ground, effective_warmup,
                                  pico_preprocessor, warmed=false]
                                (gmr::BodyMap& bones, double frame_time) mutable {
            for (const auto& name : required_bones)
                if (!bones.count(name)) throw std::runtime_error("Required source bone missing: " + name);
            for (auto& bone : bones) {
                auto& data = bone.second;
                if (!data.position.allFinite() || !data.rot_wxyz.allFinite() || data.rot_wxyz.norm() < 1e-8)
                    throw std::runtime_error("Invalid source bone pose: " + bone.first);
                data.rot_wxyz.normalize();
            }
            if (pico_preprocessor) pico_preprocessor->process(bones, frame_time);
            if (!spine.empty() && spine_deg != 0) {
                auto it = bones.find(spine);
                if (it != bones.end()) {
                    const auto& v = it->second.rot_wxyz;
                    Eigen::Quaterniond q(v[0], v[1], v[2], v[3]);
                    q = q * Eigen::Quaterniond(Eigen::AngleAxisd(spine_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()));
                    it->second.rot_wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
                }
            }
            if (!warmed) {
                for (int i = 0; i < effective_warmup && rclcpp::ok(); ++i)
                    solver_->retarget(bones, effective_ground, frame_time);
                warmed = true;
            }
        });
        reader_ = createReader();
        publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/noetix/gmt", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
        // Connect only after all model/config/parameter validation has succeeded.
        start_time_ = steadySeconds();
        reader_->connect();
        buffer_->startAsync(queue_, solver_.get());
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / hz)), [this] { publishFrame(); });
        RCLCPP_INFO(get_logger(), "device=%s robot=%s topic=%s rate=%.1fHz model=%s IK=%s",
                    device_.c_str(), robot_.c_str(), publisher_->get_topic_name(), hz, xml.c_str(), ik.c_str());
        if (sim2real_profile_)
            RCLCPP_INFO(get_logger(), "Pico sim2real IK active: 30-frame ground calibration; "
                        "per-frame grounding, foot lock, sole alignment and warmup disabled; FIFO input");
    }
    ~RetargetNode() override {
        if (timer_) timer_->cancel();
        if (buffer_) buffer_->stopAsync();
        if (reader_) reader_->disconnect();
    }
private:
    template<typename T> T parameter(const std::string& name, const T& value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;  // Device/model changes require restarting the pipeline.
        return declare_parameter<T>(name, value, descriptor);
    }
    double finite(const std::string& name, double value) {
        const auto result = parameter<double>(name, value);
        if (!std::isfinite(result)) throw std::invalid_argument(name + " must be finite");
        return result;
    }
    double positive(const std::string& name, double value) {
        auto v = finite(name, value);
        if (v <= 0) throw std::invalid_argument(name + " must be positive");
        return v;
    }
    double nonnegative(const std::string& name, double value) {
        auto v = finite(name, value);
        if (v < 0) throw std::invalid_argument(name + " must be nonnegative");
        return v;
    }
    std::unique_ptr<gmr::BaseReader> createReader() {
        if (device_ == "xsens") {
            gmr::XsensReader::Config cfg;
            cfg.port = parameter<int>("xsens_port", 9763);
            if (cfg.port < 1 || cfg.port > 65535) throw std::invalid_argument("Invalid UDP port");
            return std::make_unique<gmr::XsensReader>(queue_, cfg);
        }
#ifdef GMR_WITH_OPTITRACK
        if (device_ == "optitrack") {
            gmr::OptiTrackReader::Config cfg;
            cfg.server_ip = parameter<std::string>("server_ip", "");
            cfg.client_ip = parameter<std::string>("client_ip", "");
            if (cfg.server_ip.empty() || cfg.client_ip.empty())
                throw std::invalid_argument("OptiTrack requires server_ip and client_ip");
            return std::make_unique<gmr::OptiTrackReader>(queue_, cfg);
        }
#endif
#ifdef GMR_WITH_FZMOTION
        if (device_ == "fzmotion") {
            gmr::FzmotionReader::Config cfg;
            cfg.server_ip = parameter<std::string>("server_ip", "");
            cfg.subject_filter = parameter<std::string>("subject_filter", "");
            cfg.position_scale = positive("position_scale", 0.001);
            if (cfg.server_ip.empty()) throw std::invalid_argument("FZMotion requires server_ip");
            return std::make_unique<gmr::FzmotionReader>(queue_, cfg);
        }
#endif
#ifdef GMR_WITH_PICO
        if (device_ == "pico") {
            gmr::PicoReader::Config cfg;
            cfg.startup_timeout_sec = positive("pico_startup_timeout", 30.0);
            cfg.normalize_initial_yaw = parameter<bool>("normalize_initial_yaw", false);
            if (sim2real_profile_) cfg.normalize_initial_yaw = false;
            return std::make_unique<gmr::PicoReader>(queue_, cfg);
        }
#endif
        throw std::invalid_argument(device_ + " SDK support was disabled at build time");
    }
    void publishFrame() {
        const double now = steadySeconds();
        const auto latest = buffer_->latestProcessedFrame();
        if (!latest) return;
        if (now - latest->frame_time > stale_timeout_) {
            if (!stale_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Motion input timed out; publication paused until fresh IK data arrives");
            }
            stale_ = true;
            return;
        }
        if (stale_) {
            output_->reset();
            RCLCPP_DEBUG(get_logger(), "Fresh motion input received; publication resumed");
            stale_ = false;
        }
        const auto frame = interpolation_delay_ > 0 ?
            buffer_->interpolatedProcessedFrame(interpolation_delay_) : latest;
        if (!frame) return;
        // 消息时间代表采样时刻；同一 IK 帧不伪装成新的参考帧推进下游历史窗口。
        if (frame->frame_time <= last_published_time_) return;
        auto values = output_->encode(tuning_->apply(frame->qpos), frame->frame_time,
                                      frame->frame_time - start_time_);
        if (!values) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid IK output; frame skipped");
            return;
        }
        std_msgs::msg::Float32MultiArray msg;
        msg.data = std::move(*values);
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label = robot_;
        msg.layout.dim[0].size = static_cast<uint32_t>(msg.data.size());
        msg.layout.dim[0].stride = static_cast<uint32_t>(msg.data.size());
        publisher_->publish(msg);
        last_published_time_ = frame->frame_time;
    }
    std::string device_, robot_;
    bool sim2real_profile_ = false;
    double start_time_=0, stale_timeout_=0.5, interpolation_delay_=0;
    double last_published_time_=0;
    bool stale_=false;
    gmr::FrameQueue queue_;
    std::unique_ptr<gmr_mink::GMR> solver_;
    std::unique_ptr<gmr::BaseReader> reader_;
    // Destroy the worker before the solver and the reader, including constructor failures.
    std::unique_ptr<gmr::MotionBuffer> buffer_;
    std::unique_ptr<gmr::MotionOutput> output_;
    std::unique_ptr<gmr::JointSpeedLimiter> tuning_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try { rclcpp::spin(std::make_shared<RetargetNode>()); }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("noetix_gmr"), "%s", e.what()); status = 1;
    }
    rclcpp::shutdown();
    return status;
}
