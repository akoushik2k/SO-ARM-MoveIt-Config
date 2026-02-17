#ifndef SO_ARM_BT_PRECISE_PICK_NODE_HPP
#define SO_ARM_BT_PRECISE_PICK_NODE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace so_arm_bt
{

class PrecisePick : public BT::AsyncActionNode
{
public:
  PrecisePick(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
    : BT::AsyncActionNode(name, config), move_group_(move_group)
  {
      RCLCPP_INFO(rclcpp::get_logger("PrecisePick"), "PrecisePick Node Initialized");
      // Explicitly set the end effector link as per user request
      eef_link_ = "gripper_frame_link"; 
      move_group_->setEndEffectorLink(eef_link_);
      RCLCPP_INFO(rclcpp::get_logger("PrecisePick"), "Using End Effector Link: %s", eef_link_.c_str());
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
             BT::InputPort<double>("offset", 0.10, "Offset above target in world Z direction (meters)") };
  }

  BT::NodeStatus tick() override
  {
      geometry_msgs::msg::PoseStamped target_pose;
      double offset = 0.10;

      if (!getInput("target_pose", target_pose)) {
          RCLCPP_ERROR(rclcpp::get_logger("PrecisePick"), "target_pose not provided");
          return BT::NodeStatus::FAILURE;
      }
      getInput("offset", offset);

      RCLCPP_INFO(rclcpp::get_logger("PrecisePick"), "Starting PrecisePick sequence to x=%.3f, y=%.3f, z=%.3f", 
                  target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

      // 1. Move to Hover Pose (Target + Offset in Z)
      auto hover_pose = target_pose;
      hover_pose.pose.position.z += offset;

      RCLCPP_INFO(rclcpp::get_logger("PrecisePick"), "Stage 1: Moving to Hover Pose (z=%.3f)", hover_pose.pose.position.z);
      
      move_group_->setMaxVelocityScalingFactor(0.8);
      move_group_->setMaxAccelerationScalingFactor(0.8);
      move_group_->setPoseTarget(hover_pose);
      
      moveit::planning_interface::MoveGroupInterface::Plan hover_plan;
      if (move_group_->plan(hover_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(rclcpp::get_logger("PrecisePick"), "Failed to plan to hover pose");
          return BT::NodeStatus::FAILURE;
      }
      move_group_->execute(hover_plan);

      // 2. Slow Cartesian Approach to Target
      RCLCPP_INFO(rclcpp::get_logger("PrecisePick"), "Stage 2: Slow Cartesian Approach to Target");
      
      move_group_->setMaxVelocityScalingFactor(0.2); // Slow approach
      move_group_->setMaxAccelerationScalingFactor(0.1);
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose.pose);
      
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      if (fraction > 0.9) {
          move_group_->execute(trajectory);
          RCLCPP_INFO(rclcpp::get_logger("PrecisePick"), "PrecisePick sequence completed successfully");
          return BT::NodeStatus::SUCCESS;
      } else {
          RCLCPP_ERROR(rclcpp::get_logger("PrecisePick"), "Cartesian approach failed (only %.2f%% achieved)", fraction * 100.0);
          return BT::NodeStatus::FAILURE;
      }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string eef_link_;
};

} // namespace so_arm_bt

#endif // SO_ARM_BT_PRECISE_PICK_NODE_HPP
