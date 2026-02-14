#pragma once

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

namespace so_arm_bt
{
    class GripperAction : public BT::SyncActionNode
    {
    public:
        using GripperCommand = control_msgs::action::GripperCommand;
        using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

        GripperAction(const std::string& name, const BT::NodeConfig& config,
                      rclcpp_action::Client<GripperCommand>::SharedPtr action_client,
                      rclcpp::Executor::SharedPtr executor,
                      double position)
            : BT::SyncActionNode(name, config), 
              action_client_(action_client), 
              executor_(executor),
              target_position_(position)
        {}

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus tick() override
        {
            if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(rclcpp::get_logger("GripperAction"), "Action server not available after 5 seconds");
                return BT::NodeStatus::FAILURE;
            }

            auto goal_msg = GripperCommand::Goal();
            goal_msg.command.position = target_position_;
            goal_msg.command.max_effort = 100.0;

            RCLCPP_INFO(rclcpp::get_logger("GripperAction"), "Sending gripper goal: position=%.3f", target_position_);

            auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
            auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
            
            // Spin executor while waiting for goal acceptance
            RCLCPP_INFO(rclcpp::get_logger("GripperAction"), "Waiting for goal acceptance...");
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (std::chrono::steady_clock::now() < deadline) {
                executor_->spin_some(std::chrono::milliseconds(10));
                if (future_goal_handle.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    break;
                }
            }
            
            if (future_goal_handle.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
            {
                 RCLCPP_ERROR(rclcpp::get_logger("GripperAction"), "Action goal send timed out");
                 return BT::NodeStatus::FAILURE;
            }
            
            auto goal_handle = future_goal_handle.get();
            if (!goal_handle)
            {
                RCLCPP_ERROR(rclcpp::get_logger("GripperAction"), "Goal was rejected by server");
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(rclcpp::get_logger("GripperAction"), "Goal accepted, waiting for result...");

            // Wait for result with executor spinning
            auto future_result = action_client_->async_get_result(goal_handle);
            deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (std::chrono::steady_clock::now() < deadline) {
                executor_->spin_some(std::chrono::milliseconds(10));
                if (future_result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    break;
                }
            }
            
            if (future_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
            {
                 RCLCPP_ERROR(rclcpp::get_logger("GripperAction"), "Action result timed out");
                 return BT::NodeStatus::FAILURE;
            }

            auto result = future_result.get();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("GripperAction"), "Gripper action succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("GripperAction"), "Gripper action failed with code: %d", static_cast<int>(result.code));
                return BT::NodeStatus::FAILURE;
            }
        }

    private:
        rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;
        rclcpp::Executor::SharedPtr executor_;
        double target_position_;
    };
    
    class OpenGripper : public GripperAction
    {
    public:
        OpenGripper(const std::string& name, const BT::NodeConfig& config,
                    rclcpp_action::Client<GripperCommand>::SharedPtr action_client,
                    rclcpp::Executor::SharedPtr executor)
            : GripperAction(name, config, action_client, executor, 0.08) // Open width
        {}
    };

    class CloseGripper : public GripperAction
    {
    public:
        CloseGripper(const std::string& name, const BT::NodeConfig& config,
                     rclcpp_action::Client<GripperCommand>::SharedPtr action_client,
                     rclcpp::Executor::SharedPtr executor)
            : GripperAction(name, config, action_client, executor, 0.0) // Closed width
        {}
    };
}
