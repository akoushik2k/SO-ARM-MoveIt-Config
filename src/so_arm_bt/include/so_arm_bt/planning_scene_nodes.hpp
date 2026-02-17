#ifndef SO_ARM_BT_PLANNING_SCENE_NODES_HPP
#define SO_ARM_BT_PLANNING_SCENE_NODES_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace so_arm_bt
{

class AttachObject : public BT::SyncActionNode
{
public:
  AttachObject(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_id"),
             BT::InputPort<std::string>("link_name", "gripper", "Link to attach the object to") };
  }

  BT::NodeStatus tick() override
  {
    std::string object_id;
    std::string link_name;
    if (!getInput("object_id", object_id) || !getInput("link_name", link_name)) {
      return BT::NodeStatus::FAILURE;
    }

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = link_name;
    attached_object.object.id = object_id;
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Note: This assumes the object is already in the scene or we are defining it here.
    // For simplicity, let's just use the ID and assume it's known.
    
    psi_.applyAttachedCollisionObject(attached_object);
    return BT::NodeStatus::SUCCESS;
  }

private:
  moveit::planning_interface::PlanningSceneInterface psi_;
};

class DetachObject : public BT::SyncActionNode
{
public:
  DetachObject(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_id") };
  }

  BT::NodeStatus tick() override
  {
    std::string object_id;
    if (!getInput("object_id", object_id)) return BT::NodeStatus::FAILURE;

    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = object_id;
    detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    
    psi_.applyAttachedCollisionObject(detach_object);
    return BT::NodeStatus::SUCCESS;
  }

private:
  moveit::planning_interface::PlanningSceneInterface psi_;
};

} // namespace so_arm_bt

#endif // SO_ARM_BT_PLANNING_SCENE_NODES_HPP
