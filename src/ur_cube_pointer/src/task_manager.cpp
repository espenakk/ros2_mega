#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_interfaces/msg/detected_cubes.hpp"
#include "std_msgs/msg/bool.hpp"
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
        // Old this->declare_parameter<double>("home_position.orientation.x", 0.38);
        // Old this->declare_parameter<double>("home_position.orientation.y", 0.92);
        this->declare_parameter<double>("home_position.orientation.x", 0.63);
        this->declare_parameter<double>("home_position.orientation.y", 0.77);
        this->declare_parameter<double>("home_position.orientation.z", -0.0001);
        this->declare_parameter<double>("home_position.orientation.w", -0.00005);

        load_home_position();

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_goal", 10);

        // Add feedback subscription
        feedback_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                goal_reached_callback(msg);
            });

        // Add controller ready subscription
        ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/controller_ready", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && state_ == INIT) {
                    RCLCPP_INFO(this->get_logger(), "Controller ready. Moving to home position");
                    send_goal(home_pose_);
                    state_ = MOVING_HOME;
                }
            });

        RCLCPP_INFO(this->get_logger(), "Task Manager initialized - waiting for controller");
    }

private:
    enum State { INIT, MOVING_HOME, SCANNING, MOVING_TO_CUBE, DONE };
    State state_;
    geometry_msgs::msg::PoseStamped home_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<custom_interfaces::msg::DetectedCubes>::SharedPtr cubes_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    std::vector<geometry_msgs::msg::PoseStamped> target_poses_;
    size_t current_target_ = 0;
    int retry_count_ = 0;

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

    void send_goal(const geometry_msgs::msg::PoseStamped& goal) {
        auto stamped_goal = goal;
        stamped_goal.header.stamp = this->now();
        stamped_goal.header.frame_id = "base_link";
        pose_pub_->publish(stamped_goal);
    }

    void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            RCLCPP_ERROR(this->get_logger(), "Movement failed. Retrying...");
            if (retry_count_++ < 3) {
                send_goal(target_poses_[current_target_]);
                return;
            } else {
                RCLCPP_FATAL(this->get_logger(), "Aborting after 3 retries");
                rclcpp::shutdown();
                return;
            }
        }

        retry_count_ = 0;  // Reset retry counter on success

        switch (state_) {
            case MOVING_HOME:
                RCLCPP_INFO(this->get_logger(), "Home reached. Starting scan");
                start_scanning();
                break;

            case MOVING_TO_CUBE:
                current_target_++;
                if (current_target_ < target_poses_.size()) {
                    RCLCPP_INFO(this->get_logger(), "Moving to next target (%zu/%zu)",
                               current_target_+1, target_poses_.size());
                    send_goal(target_poses_[current_target_]);
                } else {
                    RCLCPP_INFO(this->get_logger(), "All cubes processed. Returning home");
                    send_goal(home_pose_);
                    state_ = DONE;
                }
                break;

            case DONE:
                RCLCPP_INFO(this->get_logger(), "Final home reached. Task complete");
                rclcpp::shutdown();
                break;

            default:
                break;
        }
    }

    void start_scanning() {
        state_ = SCANNING;
        RCLCPP_INFO(this->get_logger(), "Scanning for cubes...");
        cubes_sub_ = this->create_subscription<custom_interfaces::msg::DetectedCubes>(
            "/detected_cubes", 10,
            [this](const custom_interfaces::msg::DetectedCubes::SharedPtr msg) {
                if (state_ != SCANNING) return;
                cubes_sub_.reset();  // Unsubscribe after first detection

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
                    RCLCPP_INFO(this->get_logger(), "No cubes found. Returning home");
                    send_goal(home_pose_);
                    state_ = DONE;
                    return;
                }

                // Create target poses
                for (const auto& cube : sorted_cubes) {
                    geometry_msgs::msg::PoseStamped pose = home_pose_;
                    pose.pose.position.x = cube.position.x;
                    pose.pose.position.y = cube.position.y;
                    pose.pose.position.z = 0.20;  // Approach height
                    target_poses_.push_back(pose);
                }
                target_poses_.push_back(home_pose_);  // Final home position

                current_target_ = 0;
                state_ = MOVING_TO_CUBE;
                RCLCPP_INFO(this->get_logger(), "Moving to first cube (%zu total)", target_poses_.size());
                send_goal(target_poses_[current_target_]);
            });
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskManager>());
    rclcpp::shutdown();
    return 0;
}