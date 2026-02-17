#ifndef SO_ARM_BT_RECOVERY_NODES_HPP
#define SO_ARM_BT_RECOVERY_NODES_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace so_arm_bt
{

class ClearOctomap : public BT::SyncActionNode
{
public:
  ClearOctomap(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<rclcpp::Node>& node)
    : BT::SyncActionNode(name, config), node_(node)
  {
    client_ = node_->create_client<std_srvs::srv::Empty>("clear_octomap");
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (!client_->wait_for_service(std::chrono::milliseconds(100))) {
      return BT::NodeStatus::FAILURE;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    client_->async_send_request(request);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
};

class GoHome : public BT::AsyncActionNode
{
public:
  GoHome(const std::string& name, const BT::NodeConfiguration& config,
         const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
    : BT::AsyncActionNode(name, config), move_group_(move_group) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    move_group_->setNamedTarget("home");
    if (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

class Wait : public BT::SyncActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("msec") };
  }

  BT::NodeStatus tick() override
  {
    int msec;
    if (!getInput("msec", msec)) return BT::NodeStatus::FAILURE;
    std::this_thread::sleep_for(std::chrono::milliseconds(msec));
    return BT::NodeStatus::SUCCESS;
  }
};


} // namespace so_arm_bt

#endif // SO_ARM_BT_RECOVERY_NODES_HPP
