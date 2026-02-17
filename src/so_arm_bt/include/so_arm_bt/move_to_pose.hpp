#ifndef SO_ARM_BT_MOVE_TO_POSE_HPP
#define SO_ARM_BT_MOVE_TO_POSE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace so_arm_bt
{

class MoveToPose : public BT::AsyncActionNode
{
public:
  MoveToPose(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
    : BT::AsyncActionNode(name, config), move_group_(move_group)
  {}


  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("target_pose"),
             BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped"),
             BT::InputPort<bool>("cartesian", false, "Use Cartesian path for straight line motion"),
             BT::InputPort<double>("speed", 1.0, "Velocity and acceleration scaling factor (0.0 - 1.0)") };
  }

  BT::NodeStatus tick() override
  {
      geometry_msgs::msg::PoseStamped pose;
      std::string target_name;
      bool use_named = false;
      bool use_cartesian = false;
      double speed = 1.0;

      getInput("cartesian", use_cartesian);
      getInput("speed", speed);

      // Apply speed scaling
      move_group_->setMaxVelocityScalingFactor(std::clamp(speed, 0.01, 1.0));
      move_group_->setMaxAccelerationScalingFactor(std::clamp(speed, 0.01, 1.0));

      if (getInput("target_pose", target_name) && !target_name.empty())
      {
        move_group_->setNamedTarget(target_name);
        use_named = true;
      }

      
      if (!use_named && getInput("pose_stamped", pose))
      {
        RCLCPP_INFO(rclcpp::get_logger("MoveToPose"), 
                    "Moving to pose (Cartesian: %s, Speed: %.2f) in frame %s: x=%.3f, y=%.3f, z=%.3f", 
                    use_cartesian ? "yes" : "no", speed,
                    pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        
        move_group_->setGoalPositionTolerance(0.02); // 2cm
        move_group_->setGoalOrientationTolerance(0.1); // ~5.7 deg
        move_group_->setPlanningTime(10.0);
        move_group_->setNumPlanningAttempts(20);
        
        if (use_cartesian) {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose.pose);
            
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            
            if (fraction > 0.9) {
                move_group_->execute(trajectory);
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"), "Cartesian path failed (only %.2f%% achieved)", fraction * 100.0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            move_group_->setPoseTarget(pose);
        }
      }


      else if (!use_named)
      {
          // Check if target_name was just empty (from blackboard)
          if(target_name.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"), "Target pose name is empty. Planning aborted.");
            return BT::NodeStatus::FAILURE;
          }
      }

      // Only plan and execute if not using Cartesian path, as Cartesian path handles its own execution
      if (!use_cartesian) {
          moveit::planning_interface::MoveGroupInterface::Plan my_plan;
          bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

          if(success) {
              move_group_->execute(my_plan);
              return BT::NodeStatus::SUCCESS;
          } else {
              return BT::NodeStatus::FAILURE;
          }
      }
      // If we reached here and use_cartesian was true, it means the Cartesian path was already handled
      // or failed within its block, so this return should not be reached.
      // If use_cartesian was false, the above block would have returned.
      // This implies an edge case where neither named target nor pose_stamped was provided,
      // and use_cartesian was false. In that case, the "No valid target provided" error
      // should have been returned earlier.
      // For safety, return FAILURE if we somehow fall through without a clear success/failure.
      return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

} // namespace so_arm_bt

#endif // SO_ARM_BT_MOVE_TO_POSE_HPP
