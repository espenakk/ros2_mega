#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_controller_node");

// Define overarching RobotController class, responsible for creating and sending movement plans to the robot
class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode(const rclcpp::NodeOptions& options)
        : Node("robot_controller_node", options),
          move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
    {
        // Initialize MoveIt interface
        move_group_interface_.setPlanningTime(10.0);
        move_group_interface_.setNumPlanningAttempts(10);
        move_group_interface_.setGoalPositionTolerance(0.01);
        move_group_interface_.setGoalOrientationTolerance(0.05);

        // Create publishers
        feedback_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
        ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/controller_ready", 10);

        // Create subscription for pose goals
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_goal", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                pose_goal_callback(msg);
            });

        // Publish ready status after initialization is complete
        ready_timer_ = this->create_wall_timer(
            1000ms, [this]() {
                if (move_group_interface_.getCurrentState() != nullptr) {
                    std_msgs::msg::Bool ready_msg;
                    ready_msg.data = true;
                    ready_pub_->publish(ready_msg);
                    RCLCPP_INFO(LOGGER, "Robot Controller initialized and ready");
                    // Stop the timer after first successful publication
                    ready_timer_->cancel();
                }
            });
    }

private:
    // Create all necessary private class variables
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::TimerBase::SharedPtr ready_timer_;

    // Callback function that is used whenever a new pose_goal is detected
    void pose_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(LOGGER, "Received new pose goal");
        std_msgs::msg::Bool result_msg;

        try {
            move_group_interface_.setPoseTarget(msg->pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            // Plan and execute
            auto const plan_success = move_group_interface_.plan(plan);
            if (plan_success) {
                auto const execute_success = move_group_interface_.execute(plan);
                result_msg.data = (execute_success == moveit::core::MoveItErrorCode::SUCCESS);
                if (result_msg.data) {
                    RCLCPP_INFO(LOGGER, "Execution successful");
                } else {
                    RCLCPP_ERROR(LOGGER, "Execution failed");
                }
            } else {
                RCLCPP_ERROR(LOGGER, "Planning failed");
                result_msg.data = false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(LOGGER, "Movement error: %s", e.what());
            result_msg.data = false;
        }

        // Publish result
        feedback_pub_->publish(result_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<RobotControllerNode>(node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}