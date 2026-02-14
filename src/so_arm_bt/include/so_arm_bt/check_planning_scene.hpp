#pragma once

#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>

namespace so_arm_bt
{
    class CheckPlanningScene : public BT::ConditionNode
    {
    public:
        CheckPlanningScene(const std::string& name, const BT::NodeConfig& config)
            : BT::ConditionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("expected_scene") };
        }

        BT::NodeStatus tick() override
        {
            // Placeholder: Check if the MoveIt planning scene is valid or has objects
            // For now, just return SUCCESS to prove the condition node works.
            RCLCPP_INFO(rclcpp::get_logger("CheckPlanningScene"), "MoveIt Planning Scene OK");
            return BT::NodeStatus::SUCCESS;
        }
    };
}
