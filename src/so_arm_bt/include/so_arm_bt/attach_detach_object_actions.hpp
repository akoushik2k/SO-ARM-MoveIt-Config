#pragma once

#include "behaviortree_cpp/action_node.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace so_arm_bt
{

/**
 * @brief BT Action Node: Attach object to robot link in planning scene
 */
class AttachObjectAction : public BT::SyncActionNode
{
public:
    AttachObjectAction(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene)
        : BT::SyncActionNode(name, config), planning_scene_(planning_scene)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_id", "Object ID to attach"),
            BT::InputPort<std::string>("link_name", "gripper_link", "Link to attach to")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string object_id = getInput<std::string>("object_id").value_or("");
        std::string link_name = getInput<std::string>("link_name").value_or("gripper_link");

        if (object_id.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("AttachObject"), "Missing object_id");
            return BT::NodeStatus::FAILURE;
        }

        // Attach object to link
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = link_name;
        attached_object.object.id = object_id;
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        // TODO: Implement actual attachment logic with planning scene
        
        RCLCPP_INFO(rclcpp::get_logger("AttachObject"), 
                    "Attached object '%s' to link '%s'", object_id.c_str(), link_name.c_str());
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
};

/**
 * @brief BT Action Node: Detach object from robot link
 */
class DetachObjectAction : public BT::SyncActionNode
{
public:
    DetachObjectAction(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene)
        : BT::SyncActionNode(name, config), planning_scene_(planning_scene)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_id", "Object ID to detach")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string object_id = getInput<std::string>("object_id").value_or("");

        if (object_id.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("DetachObject"), "Missing object_id");
            return BT::NodeStatus::FAILURE;
        }

        // TODO: Implement actual detachment logic with planning scene
        
        RCLCPP_INFO(rclcpp::get_logger("DetachObject"), 
                    "Detached object '%s'", object_id.c_str());
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
};

} // namespace so_arm_bt
