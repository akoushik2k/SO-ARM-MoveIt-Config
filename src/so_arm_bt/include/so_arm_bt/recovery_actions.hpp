#pragma once

#include "behaviortree_cpp/action_node.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace so_arm_bt
{

/**
 * @brief BT Action Node: Return robot to home position
 */
class GoHomeAction : public BT::SyncActionNode
{
public:
    GoHomeAction(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
        : BT::SyncActionNode(name, config), move_group_(move_group)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("home_position", "Home", "Named target for home position")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string home_name = getInput<std::string>("home_position").value_or("Home");
        
        RCLCPP_INFO(rclcpp::get_logger("GoHome"), "Returning to home position: %s", home_name.c_str());
        
        move_group_->setNamedTarget(home_name);
        moveit::core::MoveItErrorCode success = move_group_->move();

        if (success == moveit::core::MoveItErrorCode::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("GoHome"), "Failed to return home");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

/**
 * @brief BT Action Node: Clear octomap (3D collision map)
 */
class ClearOctomapAction : public BT::SyncActionNode
{
public:
    ClearOctomapAction(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        // Create service client for clearing octomap
        clear_octomap_client_ = node_->create_client<std_srvs::srv::Empty>("clear_octomap");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        if (!clear_octomap_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(rclcpp::get_logger("ClearOctomap"), "clear_octomap service not available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto future = clear_octomap_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("ClearOctomap"), "Octomap cleared");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_client_;
};

} // namespace so_arm_bt
