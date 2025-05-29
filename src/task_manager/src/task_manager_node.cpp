#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_interfaces/msg/detected_cubes.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

// TaskManager class, responsible for executing the program flow
class TaskManager : public rclcpp::Node {
public:
    TaskManager() : Node("task_manager"), state_(INIT), current_backup_index_(0) {
        // Declare parameters for home position
        this->declare_parameter<std::string>("home_position.frame_id", "base_link");
        this->declare_parameter<double>("home_position.position.x", -0.253);
        this->declare_parameter<double>("home_position.position.y", 0.44);
        this->declare_parameter<double>("home_position.position.z", 0.49);
        this->declare_parameter<double>("home_position.orientation.x", 0.63);
        this->declare_parameter<double>("home_position.orientation.y", 0.77);
        this->declare_parameter<double>("home_position.orientation.z", -0.0001);
        this->declare_parameter<double>("home_position.orientation.w", -0.00005);

        // Declare backup positions parameters
        this->declare_parameter<std::vector<double>>("backup_positions.position1", {-0.22, 0.6, 0.49});
        this->declare_parameter<std::vector<double>>("backup_positions.position2", {-0.68, 0.25, 0.49});
        this->declare_parameter<std::vector<double>>("backup_positions.position3", {-0.42, -0.007, 0.49});

        load_home_position();
        load_backup_positions();

        // Add a pose publisher
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
                    state_ = MOVING_HOME;
                    send_goal(home_pose_);
                }
            });

        RCLCPP_INFO(this->get_logger(), "Task Manager initialized - waiting for controller");
    }

private:
    // Create all necessary private class variables
    enum State { INIT, MOVING_HOME, SCANNING_AT_HOME,
                 MOVING_TO_BACKUP, SCANNING_AT_BACKUP, MOVING_TO_CUBE, DONE };
    State state_;
    geometry_msgs::msg::PoseStamped home_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> backup_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<custom_interfaces::msg::DetectedCubes>::SharedPtr cubes_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    rclcpp::TimerBase::SharedPtr scan_timeout_timer_;
    std::vector<geometry_msgs::msg::PoseStamped> target_poses_;
    size_t current_target_ = 0;
    size_t current_backup_index_ = 0;
    int retry_count_ = 0;

    // Function to load home position
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
    // Function to handle moving to backup positions logic
    void load_backup_positions() {
        try {
            // Load three backup positions
            for (int i = 1; i <= 3; i++) {
                std::string param_name = "backup_positions.position" + std::to_string(i);
                auto position = this->get_parameter(param_name).as_double_array();

                if (position.size() != 3) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid backup position %d. Expected 3 values, got %zu",
                                i, position.size());
                    continue;
                }

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.pose = home_pose_.pose;  // Same orientation as home
                pose.pose.position.x = position[0];
                pose.pose.position.y = position[1];
                pose.pose.position.z = position[2];

                backup_poses_.push_back(pose);
                RCLCPP_INFO(this->get_logger(), "Loaded backup position %d: (%.3f, %.3f, %.3f)",
                           i, position[0], position[1], position[2]);
            }

            if (backup_poses_.size() < 3) {
                RCLCPP_WARN(this->get_logger(), "Only %zu backup positions loaded. Using default positions",
                           backup_poses_.size());

                // Add default positions if not enough were loaded from .yaml
                while (backup_poses_.size() < 3) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "base_link";
                    pose.pose = home_pose_.pose;

                    switch(backup_poses_.size()) {
                        case 0:
                            pose.pose.position.x = -0.22;
                            pose.pose.position.y = 0.6;
                            break;
                        case 1:
                            pose.pose.position.x = -0.68;
                            pose.pose.position.y = 0.25;
                            break;
                        case 2:
                            pose.pose.position.x = -0.42;
                            pose.pose.position.y = 0.007;
                            break;
                    }

                    backup_poses_.push_back(pose);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Backup position error: %s. Using defaults", e.what());

            // Create default backup positions, in case .yaml parameters are not loaded
            backup_poses_.resize(3);
            for (auto& pose : backup_poses_) {
                pose.header.frame_id = "base_link";
                pose.pose = home_pose_.pose;
            }

            backup_poses_[0].pose.position.x = -0.22;
            backup_poses_[0].pose.position.y = 0.6;

            backup_poses_[1].pose.position.x = -0.68;
            backup_poses_[1].pose.position.y = 0.25;

            backup_poses_[2].pose.position.x = -0.42;
            backup_poses_[2].pose.position.y = 0.007;
        }
    }

    // Function to send a new goal to the robot planner
    void send_goal(const geometry_msgs::msg::PoseStamped& goal) {
        auto stamped_goal = goal;
        stamped_goal.header.stamp = this->now();
        stamped_goal.header.frame_id = "base_link";
        pose_pub_->publish(stamped_goal);
    }

    // Switch-case logic to decide where the robot should move
    void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            RCLCPP_ERROR(this->get_logger(), "Movement failed. Retrying...");
            if (retry_count_++ < 3) {
                // Resend current goal based on state
                switch (state_) {
                    case MOVING_HOME:
                        send_goal(home_pose_);
                        break;
                    case MOVING_TO_BACKUP:
                        send_goal(backup_poses_[current_backup_index_]);
                        break;
                    case MOVING_TO_CUBE:
                        if (current_target_ < target_poses_.size()) {
                            send_goal(target_poses_[current_target_]);
                        }
                        break;
                    default:
                        RCLCPP_WARN(this->get_logger(), "Retry not implemented for current state");
                        break;
                }
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
                state_ = SCANNING_AT_HOME;
                start_scanning();
                break;

            case MOVING_TO_BACKUP:
                RCLCPP_INFO(this->get_logger(), "Backup location %zu reached. Starting scan",
                           current_backup_index_ + 1);
                state_ = SCANNING_AT_BACKUP;
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
                RCLCPP_WARN(this->get_logger(), "Unexpected state in goal reached callback");
                break;
        }
    }

    // Scanning function, responsible for retrieving the data from the camera node
    // and formating it in a way that is useful for decision making
    void start_scanning() {
    RCLCPP_INFO(this->get_logger(), "Scanning for cubes...");

    // Cancel previous timer if any
    if (scan_timeout_timer_) {
        scan_timeout_timer_->cancel();
    }

    // Start timeout timer, set to 5 seconds
    scan_timeout_timer_ = this->create_wall_timer(5s, [this]() {
        RCLCPP_WARN(this->get_logger(), "Scan timeout. No cubes detected.");
        scan_timeout_timer_->cancel();  // Stop the timer

        if (state_ == SCANNING_AT_HOME) {
            current_backup_index_ = 0;
            state_ = MOVING_TO_BACKUP;
            RCLCPP_INFO(this->get_logger(), "Moving to backup location 1");
            send_goal(backup_poses_[current_backup_index_]);
        }
        else if (state_ == SCANNING_AT_BACKUP) {
            current_backup_index_++;
            if (current_backup_index_ < backup_poses_.size()) {
                RCLCPP_INFO(this->get_logger(), "Moving to backup location %zu",
                            current_backup_index_ + 1);
                state_ = MOVING_TO_BACKUP;
                send_goal(backup_poses_[current_backup_index_]);
            } else {
                RCLCPP_INFO(this->get_logger(), "No cubes found after all backup locations. Returning home");
                send_goal(home_pose_);
                state_ = DONE;
            }
        }
    });

    // Subscribe to detected cubes
    cubes_sub_ = this->create_subscription<custom_interfaces::msg::DetectedCubes>(
        "/detected_cubes", 10,
        [this](const custom_interfaces::msg::DetectedCubes::SharedPtr msg) {
            // Only process if scanning
            if (state_ != SCANNING_AT_HOME && state_ != SCANNING_AT_BACKUP) {
                RCLCPP_WARN(this->get_logger(), "Received cubes message but not in scanning state. Ignoring.");
                return;
            }

            // Cancel scan timer and unsubscribe
            if (scan_timeout_timer_) {
                scan_timeout_timer_->cancel();
            }
            cubes_sub_.reset();

            std::vector<custom_interfaces::msg::DetectedCube> sorted_cubes;
            // Sort: red -> blue -> yellow
            // Done by pushing new cubes to the back of the list
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
                RCLCPP_INFO(this->get_logger(), "No cubes detected in message. Waiting for more...");
                return;
            }

            // Found cubes, prepare target poses
            target_poses_.clear();
            for (const auto& cube : sorted_cubes) {
                geometry_msgs::msg::PoseStamped pose = home_pose_;
                pose.pose.position.x = cube.position.x;
                pose.pose.position.y = cube.position.y;
                pose.pose.position.z = 0.15;  // Approach height
                target_poses_.push_back(pose);
            }
            target_poses_.push_back(home_pose_);  // Return home after last cube

            current_target_ = 0;
            state_ = MOVING_TO_CUBE;
            RCLCPP_INFO(this->get_logger(), "%zu cubes detected. Moving to first cube", sorted_cubes.size());
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