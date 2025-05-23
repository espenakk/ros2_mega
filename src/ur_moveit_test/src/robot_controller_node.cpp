
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For quaternion conversion
#include <tf2_eigen/tf2_eigen.hpp> // For Eigen to geometry_msgs conversion

// Define a logger for this node
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_controller_node");

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode(const rclcpp::NodeOptions& options)
        : Node("robot_controller_node", options),
          move_group_interface_(std::shared_ptr<rclcpp::Node>(this), PLANNING_GROUP_ARM) {
        
        RCLCPP_INFO(LOGGER, "Robot Controller Node initialized.");
        RCLCPP_INFO(LOGGER, "Planning group: %s", PLANNING_GROUP_ARM.c_str());
        RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_interface_.getEndEffectorLink().c_str());
        RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_interface_.getPlanningFrame().c_str());

        // Example: Move to a predefined pose
        // You might want to trigger this from a service, action, or topic in a real application
        // For simplicity, we'll call it directly after a short delay.
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), // Wait 5 seconds for MoveIt to fully initialize
            std::bind(&RobotControllerNode::moveToPoseGoal, this));
    }

    void moveToPoseGoal() {
        // Cancel the timer so it only runs once
        if (timer_) {
            timer_->cancel();
        }

        RCLCPP_INFO(LOGGER, "Attempting to move to a predefined pose...");

        // Set a target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 1, 0), M_PI)); // Example: Rotate 180 deg around Y (pointing down)
        // For UR3, a common pose might be:
        target_pose.position.x = 0.3;
        target_pose.position.y = 0.1;
        target_pose.position.z = 0.4; // Adjust these values based on your robot's reach and workspace

        move_group_interface_.setPoseTarget(target_pose);
        move_group_interface_.setPlanningTime(10.0); // seconds
        move_group_interface_.setNumPlanningAttempts(5);
        move_group_interface_.setGoalPositionTolerance(0.01); // meters
        move_group_interface_.setGoalOrientationTolerance(0.01); // radians


        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(LOGGER, "Planning successful. Executing trajectory...");
            // Execute
            moveit::core::MoveItErrorCode execute_status = move_group_interface_.execute(my_plan);
            if (execute_status == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(LOGGER, "Execution successful!");
            } else {
                RCLCPP_ERROR(LOGGER, "Execution failed with error code: %d", execute_status.val);
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Planning failed!");
        }
    }

    void moveToJointGoal() {
        RCLCPP_INFO(LOGGER, "Attempting to move to a predefined joint configuration...");

        // Get current joint values
        std::vector<double> current_joints = move_group_interface_.getCurrentJointValues();
        
        // You can print them if you want
        // for(size_t i = 0; i < current_joints.size(); ++i) {
        //     RCLCPP_INFO(LOGGER, "Joint %zu: %f", i, current_joints[i]);
        // }

        // Set a target joint configuration (example for a 6-DOF arm)
        // Make sure these values are within your robot's joint limits!
        std::vector<double> target_joint_values = {0.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, -M_PI/2.0, 0.0}; 
        // This is just an example, adjust for your UR3 (e.g. shoulder_pan, shoulder_lift, elbow_joint, etc.)

        if (target_joint_values.size() != move_group_interface_.getVariableCount()) {
            RCLCPP_ERROR(LOGGER, "Target joint values count (%zu) does not match robot's joint count (%u).",
                         target_joint_values.size(), move_group_interface_.getVariableCount());
            return;
        }

        move_group_interface_.setJointValueTarget(target_joint_values);
        move_group_interface_.setPlanningTime(10.0);
        move_group_interface_.setNumPlanningAttempts(5);
        move_group_interface_.setGoalJointTolerance(0.01); // radians

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(LOGGER, "Joint goal planning successful. Executing trajectory...");
            if (move_group_interface_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(LOGGER, "Joint goal execution successful!");
            } else {
                RCLCPP_ERROR(LOGGER, "Joint goal execution failed!");
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Joint goal planning failed!");
        }
    }


private:
    // For UR robots, the planning group is often "manipulator" or "ur_manipulator"
    // Check your SRDF file for the correct group name.
    const std::string PLANNING_GROUP_ARM = "ur_manipulator"; // Or "manipulator" - VERY IMPORTANT
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Set up node options for intra-process communication (recommended for MoveIt)
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);

    auto robot_controller_node = std::make_shared<RobotControllerNode>(node_options);
    
    // Use a MultiThreadedExecutor to allow MoveIt to process callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot_controller_node);
    RCLCPP_INFO(LOGGER, "Spinning Robot Controller Node...");
    executor.spin(); // Spin the node to keep it alive and process callbacks

    rclcpp::shutdown();
    return 0;
}