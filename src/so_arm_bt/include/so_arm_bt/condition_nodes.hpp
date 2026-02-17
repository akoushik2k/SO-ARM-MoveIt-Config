#ifndef SO_ARM_BT_CONDITION_NODES_HPP
#define SO_ARM_BT_CONDITION_NODES_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

namespace so_arm_bt
{

class AtPose : public BT::ConditionNode
{
public:
  AtPose(const std::string& name, const BT::NodeConfiguration& config,
         const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
    : BT::ConditionNode(name, config), move_group_(move_group) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
             BT::InputPort<double>("tolerance", 0.05, "L2 distance tolerance") };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped target_pose;
    double tolerance;
    if (!getInput("target_pose", target_pose) || !getInput("tolerance", tolerance)) {
      return BT::NodeStatus::FAILURE;
    }

    auto current_pose = move_group_->getCurrentPose();
    
    double dx = current_pose.pose.position.x - target_pose.pose.position.x;
    double dy = current_pose.pose.position.y - target_pose.pose.position.y;
    double dz = current_pose.pose.position.z - target_pose.pose.position.z;
    
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    return (dist < tolerance) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

} // namespace so_arm_bt

#endif // SO_ARM_BT_CONDITION_NODES_HPP
