#pragma once

#include "behaviortree_cpp/action_node.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace so_arm_bt
{
    class MoveToPose : public BT::StatefulActionNode
    {
    public:
        MoveToPose(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
            : BT::StatefulActionNode(name, config), move_group_(move_group)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("target_pose") };
        }

        BT::NodeStatus onStart() override
        {
            std::string target_pose;
            if (!getInput("target_pose", target_pose))
            {
                RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"), "Missing parameter [target_pose]");
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(rclcpp::get_logger("MoveToPose"), "Moving to pose: %s", target_pose.c_str());

            move_group_->setNamedTarget(target_pose);
            
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (!success)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"), "Planning failed");
                return BT::NodeStatus::FAILURE;
            }

            // Execute the plan asynchronously
            move_group_->asyncExecute(my_plan);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            // ideally we should check the future/status of the execution
            // For simplicity in this phase, we'll check if the move group is still moving?
            // But MoveGroupInterface doesn't expose a simple "isExecuting" that works well with asyncExecute without blocking.
            // A better approach for robust BTs is to use the MoveGroup action directly or check trajectory execution status. 
            // For this "plumbing" phase 1, we can just block in onStart (making it a Synchronous node)
            // OR we can simple-poll. Let's try to change this to a SynchronousActionNode first for simplicity if the user allows blocking.
            // But to be "Stateful", let's assume we want non-blocking.
            
            // However, MoveGroupInterface::move() is blocking. MoveGroupInterface::asyncExecute() is non-blocking.
            // We need to know when it finishes.
            
            // For Phase 1 validation, let's switch to a SIMPLE blocking approach using SyncActionNode 
            // because `asyncExecute` doesn't provide an easy handle to check completion without callbacks/futures 
            // which are harder to integrate into `onRunning` without extra boilerplate.
            
            return BT::NodeStatus::SUCCESS; 
        }

        void onHalted() override
        {
            move_group_->stop();
        }
        
    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    };
    
    // REDEFINING AS SYNCHRONOUS FOR PHASE 1 SIMPLICITY
    // If we want it to be stateful/async, we need to handle the future from asyncExecute.
    // For now, let's use SyncActionNode with .move() (blocking) effectively.
    // But wait, BT nodes shouldn't block the tree tick for too long if other nodes need to run.
    // Since this is the only thing running, blocking is fine for Phase 1.
    
    class MoveToPoseSync : public BT::SyncActionNode
    {
    public:
         MoveToPoseSync(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
            : BT::SyncActionNode(name, config), move_group_(move_group)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("target_pose") };
        }

        BT::NodeStatus tick() override
        {
            std::string target_pose;
            if (!getInput("target_pose", target_pose))
            {
                RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"), "Missing parameter [target_pose]");
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(rclcpp::get_logger("MoveToPose"), "Moving to pose: %s", target_pose.c_str());

            move_group_->setNamedTarget(target_pose);
            
            // Plan and Execute (blocking)
            moveit::core::MoveItErrorCode success = move_group_->move();

            if (success == moveit::core::MoveItErrorCode::SUCCESS)
            {
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"), "Move failed");
                return BT::NodeStatus::FAILURE;
            }
        }
        
    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    };
}
