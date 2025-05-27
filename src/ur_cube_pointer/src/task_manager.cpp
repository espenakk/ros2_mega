#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_interfaces/msg/detected_cubes.hpp"
#include <vector>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class TaskManager : public rclcpp::Node {
public:
    TaskManager() : Node("task_manager"), state_(INIT) {
        // Declare parameters
        this->declare_parameter<std::string>("home_position.frame_id", "base_link");
        this->declare_parameter<double>("home_position.position.x", -0.253);
        this->declare_parameter<double>("home_position.position.y", 0.44);
        this->declare_parameter<double>("home_position.position.z", 0.49);
        this->declare_parameter<double>("home_position.orientation.x", 0.38);
        this->declare_parameter<double>("home_position.orientation.y", 0.92);
        this->declare_parameter<double>("home_position.orientation.z", -0.00001);
        this->declare_parameter<double>("home_position.orientation.w", -0.00005);

        load_home_position();

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_goal", 10);

        timer_ = this->create_wall_timer(1s, [this]() {
            if (state_ == INIT) {
                send_home();
                state_ = MOVING_HOME;
                timer_->cancel();
                timer_ = this->create_wall_timer(5s, [this]() {
                    RCLCPP_INFO(this->get_logger(), "Starting cube detection.");
                    state_ = SCANNING;
                    cubes_sub_ = this->create_subscription<custom_interfaces::msg::DetectedCubes>(
                        "/detected_cubes", 10,
                        [this](const custom_interfaces::msg::DetectedCubes::SharedPtr msg) {
                            detected_cubes_callback(msg);
                        });
                    timer_->cancel();
                });
            }
        });
    }

private:
    enum State { INIT, MOVING_HOME, SCANNING, PROCESSING, RETURNING_HOME };
    State state_;
    geometry_msgs::msg::PoseStamped home_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<custom_interfaces::msg::DetectedCubes>::SharedPtr cubes_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> target_poses_;
    size_t current_target_ = 0;

    void load_home_position() {
        try {
            home_pose_.header.frame_id = this->get_parameter("home_position.frame_id").as_string();
            home_pose_.pose.position.x = this->get_parameter("home_position.position.x").as_double();
            home_pose_.pose.position.y = this->get_parameter("home_position.position.y").as_double();
            home_pose_.pose.position.z = this->get_parameter("home_position.position.z").as_double();
            home_pose_.pose.orientation.x = this->get_parameter("home_position.orientation.x").as_double();
            home_pose_.pose.orientation.y = this->get_parameter("home_position.orientation.y").as_double();
            home_pose_.pose.orientation.z = this->get_parameter("home_position.orientation.z").as_double();
            home_pose_.pose.orientation.w = this->get_parameter("home_position.orientation.w").as_double();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void send_home() {
        home_pose_.header.stamp = this->now();
        pose_pub_->publish(home_pose_);
        RCLCPP_INFO(this->get_logger(), "Home position sent.");
    }

    void detected_cubes_callback(const custom_interfaces::msg::DetectedCubes::SharedPtr msg) {
        if (state_ != SCANNING) return;

        cubes_sub_.reset();
        std::vector<custom_interfaces::msg::DetectedCube> sorted_cubes;

        // Sort cubes: red -> blue -> yellow
        for (const auto& cube : msg->cubes) {
            if (cube.color == "red") sorted_cubes.push_back(cube);
        }
        for (const auto& cube : msg->cubes) {
            if (cube.color == "blue") sorted_cubes.push_back(cube);
        }
        for (const auto& cube : msg->cubes) {
            if (cube.color == "yellow") sorted_cubes.push_back(cube);
        }

        if (sorted_cubes.empty()) {
            RCLCPP_INFO(this->get_logger(), "No cubes found. Returning home.");
            send_home();
            return;
        }

        // Create target poses
        for (const auto& cube : sorted_cubes) {
            geometry_msgs::msg::PoseStamped pose = home_pose_;
            pose.pose.position.x = cube.position.x;
            pose.pose.position.y = cube.position.y;
            pose.pose.position.z = 0.15;
            target_poses_.push_back(pose);
        }
        target_poses_.push_back(home_pose_);

        state_ = PROCESSING;
        current_target_ = 0;
        send_next_target();
    }

    void send_next_target() {
        if (current_target_ >= target_poses_.size()) {
            RCLCPP_INFO(this->get_logger(), "Task complete.");
            state_ = INIT;
            return;
        }

        auto pose = target_poses_[current_target_];
        pose.header.stamp = this->now();
        pose_pub_->publish(pose);
        RCLCPP_INFO(this->get_logger(), "Target %zu sent.", current_target_);

        timer_ = this->create_wall_timer(5s, [this]() {
            current_target_++;
            send_next_target();
        });
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskManager>());
    rclcpp::shutdown();
    return 0;
}