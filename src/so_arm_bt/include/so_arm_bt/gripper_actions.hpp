#ifndef SO_ARM_BT_GRIPPER_ACTIONS_HPP
#define SO_ARM_BT_GRIPPER_ACTIONS_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace so_arm_bt
{

class GripperAction : public BT::AsyncActionNode
{
public:
  GripperAction(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
    : BT::AsyncActionNode(name, config), move_group_(move_group)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("state", "open", "Desired state: open or close") };
  }

  BT::NodeStatus tick() override
  {
    std::string state;
    if (!getInput("state", state))
    {
      throw BT::RuntimeError("missing required input [state]");
    }

    if (state == "open")
    {
      move_group_->setNamedTarget("open"); // Assuming 'open' is a named target in SRDF
    }
    else if (state == "close")
    {
      move_group_->setNamedTarget("close"); // Assuming 'close' named target
    }
    else
    {
       // Direct joint value fallback if needed, or error
       return BT::NodeStatus::FAILURE;
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group_->execute(my_plan);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

} // namespace so_arm_bt

#endif // SO_ARM_BT_GRIPPER_ACTIONS_HPP
