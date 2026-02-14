#pragma once

#include "behaviortree_cpp/action_node.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace so_arm_bt
{
    class GripperMoveItAction : public BT::SyncActionNode
    {
    public:
        GripperMoveItAction(const std::string& name, const BT::NodeConfig& config,
                            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group,
                            double position)
            : BT::SyncActionNode(name, config), gripper_group_(gripper_group), target_position_(position)
        {}

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus tick() override
        {
            RCLCPP_INFO(rclcpp::get_logger("GripperMoveItAction"), "Setting gripper position: %.3f", target_position_);

            // Set joint target for gripper
            std::vector<double> joint_values = {target_position_};
            gripper_group_->setJointValueTarget(gripper_group_->getJointNames()[0], target_position_);
            
            // Plan and execute
            moveit::core::MoveItErrorCode success = gripper_group_->move();

            if (success == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("GripperMoveItAction"), "Gripper move succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("GripperMoveItAction"), "Gripper move failed");
                return BT::NodeStatus::FAILURE;
            }
        }

    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
        double target_position_;
    };
    
    class OpenGripperMoveIt : public GripperMoveItAction
    {
    public:
        OpenGripperMoveIt(const std::string& name, const BT::NodeConfig& config,
                          std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group)
            : GripperMoveItAction(name, config, gripper_group, 0.08) // Open width
        {}
    };

    class CloseGripperMoveIt : public GripperMoveItAction
    {
    public:
        CloseGripperMoveIt(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group)
            : GripperMoveItAction(name, config, gripper_group, 0.0) // Closed width
        {}
    };
}
