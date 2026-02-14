#pragma once

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include "so_arm_bt_interfaces/srv/detect_objects.hpp"

namespace so_arm_bt
{

/**
 * @brief BT Action Node: Detect objects using vision system
 * 
 * Input Ports:
 *   - object_class: (optional) Filter by object class name
 *   - min_confidence: (optional) Minimum detection confidence (default: 0.5)
 *   - output_key: Blackboard key to store detections
 * 
 * Returns SUCCESS if objects detected, FAILURE otherwise
 */
class DetectObjectsAction : public BT::SyncActionNode
{
public:
    DetectObjectsAction(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node,
        rclcpp::Executor::SharedPtr executor)
        : BT::SyncActionNode(name, config), node_(node), executor_(executor)
    {
        client_ = node_->create_client<so_arm_bt_interfaces::srv::DetectObjects>("detect_objects");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_class", "", "Object class to detect (empty = all)"),
            BT::InputPort<float>("min_confidence", 0.5f, "Minimum confidence threshold"),
            BT::OutputPort<vision_msgs::msg::Detection3DArray>("detections", "Detected objects with 3D poses")
        };
    }

    BT::NodeStatus tick() override
    {
        // Get parameters
        std::string object_class = getInput<std::string>("object_class").value_or("");
        float min_confidence = getInput<float>("min_confidence").value_or(0.5f);

        RCLCPP_INFO(node_->get_logger(), "DetectObjects: Calling service for class='%s', min_conf=%.2f",
                   object_class.c_str(), min_confidence);

        // Wait for service
        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "DetectObjects service not available");
            return BT::NodeStatus::FAILURE;
        }

        // Create request
        auto request = std::make_shared<so_arm_bt_interfaces::srv::DetectObjects::Request>();
        request->object_class = object_class;
        request->min_confidence = min_confidence;

        // Call service asynchronously
        auto future = client_->async_send_request(request);
        
        // Poll for response while spinning executor to process callbacks
        auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < timeout) {
            // Spin executor to process service response
            executor_->spin_some();
            
            auto status = future.wait_for(std::chrono::milliseconds(10));
            if (status == std::future_status::ready) {
                break;
            }
        }
        
        // Check if we got a response
        if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "DetectObjects service call timeout");
            return BT::NodeStatus::FAILURE;
        }

        auto response = future.get();
        
        if (!response->success || response->detections.detections.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No objects detected: %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Store detections in blackboard output port
        setOutput("detections", response->detections);

        RCLCPP_INFO(node_->get_logger(), "Detected %zu objects", response->detections.detections.size());
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    rclcpp::Client<so_arm_bt_interfaces::srv::DetectObjects>::SharedPtr client_;
};

} // namespace so_arm_bt
