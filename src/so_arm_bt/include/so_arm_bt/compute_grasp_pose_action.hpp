#pragma once

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include "so_arm_bt_interfaces/srv/compute_grasp_pose.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace so_arm_bt
{

/**
 * @brief BT Action Node: Compute grasp pose for detected object
 * 
 * Input Ports:
 *   - object_id: ID of object to grasp
 *   - object_pose: (optional) Pose of object if already known
 *   - output_key: Blackboard key to store grasp pose
 * 
 * Returns SUCCESS if valid grasp computed, FAILURE otherwise
 */
class ComputeGraspPoseAction : public BT::SyncActionNode
{
public:
    ComputeGraspPoseAction(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        client_ = node_->create_client<so_arm_bt_interfaces::srv::ComputeGraspPose>("compute_grasp_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_id", "Object ID to grasp"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("object_pose", "Object pose"),
            BT::OutputPort<std::string>("grasp_output_key", "grasp_pose", "Key for grasp pose"),
            BT::OutputPort<std::string>("pre_grasp_output_key", "pre_grasp_pose", "Key for pre-grasp pose")
        };
    }

    BT::NodeStatus tick() override
    {
        // Get object pose from input
        auto object_pose_opt = getInput<geometry_msgs::msg::PoseStamped>("object_pose");
        
        if (!object_pose_opt) {
            RCLCPP_ERROR(node_->get_logger(), "Missing object_pose input");
            return BT::NodeStatus::FAILURE;
        }

        // Wait for service
        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "ComputeGraspPose service not available");
            return BT::NodeStatus::FAILURE;
        }

        // Create request
        auto request = std::make_shared<so_arm_bt_interfaces::srv::ComputeGraspPose::Request>();
        request->object_id = getInput<std::string>("object_id").value_or("unknown");
        request->object_pose = object_pose_opt.value();

        // Call service
        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "ComputeGraspPose service call failed");
            return BT::NodeStatus::FAILURE;
        }

        auto response = future.get();
        
        if (!response->success) {
            RCLCPP_ERROR(node_->get_logger(), "Grasp computation failed: %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Store grasp poses in blackboard
        std::string grasp_key = getInput<std::string>("grasp_output_key").value_or("grasp_pose");
        std::string pre_grasp_key = getInput<std::string>("pre_grasp_output_key").value_or("pre_grasp_pose");
        
        setOutput(grasp_key, response->grasp_pose);
        setOutput(pre_grasp_key, response->pre_grasp_pose);

        RCLCPP_INFO(node_->get_logger(), "Grasp pose computed (quality: %.2f)", response->quality_score);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<so_arm_bt_interfaces::srv::ComputeGraspPose>::SharedPtr client_;
};

} // namespace so_arm_bt
