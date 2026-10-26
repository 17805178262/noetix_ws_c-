#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <nlohmann/json.hpp>
#include "gmr/viewer_pose.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

class MujocoViewer : public rclcpp::Node {
public:
    MujocoViewer() : Node("noetix_gmr_viewer") {
        robot_ = declare_parameter<std::string>("robot", "n3");
        preset_ = gmr::robotPreset(robot_);
        auto path = declare_parameter<std::string>("model_path", "");
        const std::filesystem::path share(ament_index_cpp::get_package_share_directory("noetix_gmr"));
        if (path.empty()) {
            std::ifstream input(share / "config/robots.json");
            nlohmann::json config; input >> config;
            path = (share / config.at(robot_).at("model").get<std::string>()).string();
        }
        const auto topic = declare_parameter<std::string>("topic", "/noetix/gmt");
        const int width = declare_parameter<int>("width", 960);
        const int height = declare_parameter<int>("height", 720);
        if (width < 320 || height < 240) throw std::invalid_argument("Window must be at least 320 x 240");
        char error[1024]{};
        model_.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
        if (!model_) throw std::runtime_error(std::string("MuJoCo model load failed: ") + error);
        if (model_->nq != 7 + preset_.num_joints || model_->njnt < 1 ||
            model_->jnt_type[0] != mjJNT_FREE || model_->jnt_qposadr[0] != 0)
            throw std::invalid_argument("Viewer model must match the robot preset and start with a free root");
        data_.reset(mj_makeData(model_.get()));
        if (!data_) throw std::runtime_error("MuJoCo data allocation failed");
        mj_forward(model_.get(), data_.get());
        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&option_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);
        resetCamera();
        subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr message) {
                if (!message->layout.dim.empty() && message->layout.dim[0].label != robot_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                        "Ignoring frame for another robot; viewer robot=%s", robot_.c_str());
                    return;
                }
                std::vector<double> pose;
                if (message->layout.data_offset != 0 || !gmr::decodeViewerPose(message->data, preset_, pose)) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid motion frame");
                    return;
                }
                std::copy(pose.begin(), pose.end(), data_->qpos);
                mj_forward(model_.get(), data_.get());
                if (!received_) RCLCPP_INFO(get_logger(), "First motion frame received (%d joints)", preset_.num_joints);
                received_ = true;
                last_frame_ = std::chrono::steady_clock::now();
            });
        RCLCPP_INFO(get_logger(), "robot=%s topic=%s model=%s", robot_.c_str(), topic.c_str(), path.c_str());
        width_ = width; height_ = height;
    }

    ~MujocoViewer() override {
        if (window_) {
            glfwMakeContextCurrent(window_);
            mjr_freeContext(&context_);
            mjv_freeScene(&scene_);
            glfwDestroyWindow(window_);
        }
        if (glfw_initialized_) glfwTerminate();
    }

    void openWindow() {
        glfwSetErrorCallback([](int, const char* description) { std::fprintf(stderr, "GLFW: %s\n", description); });
        if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; run from a graphical desktop with DISPLAY set");
        glfw_initialized_ = true;
        window_ = glfwCreateWindow(width_, height_, ("noetix_gmr | " + robot_).c_str(), nullptr, nullptr);
        if (!window_) throw std::runtime_error("Could not create MuJoCo window");
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        mjv_makeScene(model_.get(), &scene_, 10000);
        mjr_makeContext(model_.get(), &context_, mjFONTSCALE_150);
        glfwSetWindowUserPointer(window_, this);
        glfwSetKeyCallback(window_, [](GLFWwindow* window, int key, int, int action, int) {
            if (action != GLFW_PRESS) return;
            auto& self = *static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
            if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
            if (key == GLFW_KEY_R) self.resetCamera();
            if (key == GLFW_KEY_F) self.follow_ = !self.follow_;
        });
        glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double x, double y) {
            auto& self = *static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
            const double dx = x-self.mouse_x_, dy = y-self.mouse_y_;
            self.mouse_x_=x; self.mouse_y_=y;
            const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            const bool middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            if (!left && !right && !middle) return;
            int w,h; glfwGetWindowSize(window, &w, &h);
            if (h <= 0) return;
            const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                               glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            const auto action = right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) :
                                left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V) : mjMOUSE_ZOOM;
            if (right) self.follow_ = false;
            mjv_moveCamera(self.model_.get(), action, dx/h, dy/h, &self.scene_, &self.camera_);
        });
        glfwSetMouseButtonCallback(window_, [](GLFWwindow* window, int, int, int) {
            auto& self = *static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
            glfwGetCursorPos(window, &self.mouse_x_, &self.mouse_y_);
        });
        glfwSetScrollCallback(window_, [](GLFWwindow* window, double, double y) {
            auto& self = *static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
            mjv_moveCamera(self.model_.get(), mjMOUSE_ZOOM, 0, -0.05*y, &self.scene_, &self.camera_);
        });
    }

    bool render() {
        if (glfwWindowShouldClose(window_)) return false;
        if (follow_) for (int i=0;i<3;++i) camera_.lookat[i] = data_->qpos[i];
        int width,height; glfwGetFramebufferSize(window_, &width, &height);
        if (width > 0 && height > 0) {
            const mjrRect viewport{0,0,width,height};
            mjv_updateScene(model_.get(), data_.get(), &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
            mjr_render(viewport, &scene_, &context_);
            const bool stale = received_ && std::chrono::duration<double>(
                std::chrono::steady_clock::now()-last_frame_).count() > 0.5;
            const char* state = !received_ ? "Waiting for /noetix/gmt" : stale ? "No fresh data - holding pose" : "Receiving motion";
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, state,
                        "Mouse: rotate / pan / zoom\nR: reset camera   F: follow   Esc: close", &context_);
            glfwSwapBuffers(window_);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        glfwPollEvents();
        return true;
    }

private:
    void resetCamera() {
        mjv_defaultCamera(&camera_);
        camera_.azimuth = 135; camera_.elevation = -15;
        camera_.distance = std::max(2.0, 1.5*model_->stat.extent);
        for (int i=0;i<3;++i) camera_.lookat[i] = data_->qpos[i];
        follow_ = true;
    }
    std::string robot_;
    gmr::RobotPreset preset_;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};
    GLFWwindow* window_ = nullptr;
    bool glfw_initialized_ = false, received_ = false, follow_ = true;
    int width_ = 960, height_ = 720;
    double mouse_x_ = 0, mouse_y_ = 0;
    std::chrono::steady_clock::time_point last_frame_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto viewer = std::make_shared<MujocoViewer>();
        viewer->openWindow();
        while (rclcpp::ok()) {
            rclcpp::spin_some(viewer);
            if (!viewer->render()) break;
        }
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("noetix_gmr_viewer"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
