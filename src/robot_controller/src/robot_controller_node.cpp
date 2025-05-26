#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("robot_controller_node", options),
      move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
    {
        RCLCPP_INFO(this->get_logger(), "Robot Controller Node initialized. Subscribing to /pose_goal");

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_goal", 10,
            std::bind(&RobotControllerNode::poseCallback, this, std::placeholders::_1));
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received pose. Executing...");
        executePose(msg->pose);
    }

    void executePose(const geometry_msgs::msg::Pose & pose) {
        move_group_interface_.setPoseTarget(pose);
        move_group_interface_.setPlanningTime(10.0);
        move_group_interface_.setNumPlanningAttempts(10);
        move_group_interface_.setGoalPositionTolerance(0.01);
        move_group_interface_.setGoalOrientationTolerance(0.01);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
            auto result = move_group_interface_.execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Execution succeeded.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Execution failed.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
