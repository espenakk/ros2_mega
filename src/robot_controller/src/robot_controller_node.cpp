#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_controller_node");

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode(const rclcpp::NodeOptions& options)
        : Node("robot_controller_node", options),
          move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
    {
        // Initialize MoveIt interface
        move_group_interface_.setPlanningTime(10.0);
        move_group_interface_.setNumPlanningAttempts(5);
        move_group_interface_.setGoalPositionTolerance(0.01);
        move_group_interface_.setGoalOrientationTolerance(0.05);

        // Create subscription for pose goals
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_goal", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                pose_goal_callback(msg);
            });

        RCLCPP_INFO(LOGGER, "Robot Controller initialized");
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

    void pose_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(LOGGER, "Received new pose goal");

        try {
            move_group_interface_.setPoseTarget(msg->pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            if (move_group_interface_.plan(plan)) {
                auto result = move_group_interface_.execute(plan);
                if(result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(LOGGER, "Execution successful");
                } else {
                    RCLCPP_ERROR(LOGGER, "Execution failed with error code: %d", result.val);
                }
            } else {
                RCLCPP_ERROR(LOGGER, "Planning failed");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(LOGGER, "Movement error: %s", e.what());
        }
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