#ifndef SO_ARM_BT_VISION_NODES_HPP
#define SO_ARM_BT_VISION_NODES_HPP

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include "so_arm_bt_interfaces/srv/detect_objects.hpp"

namespace so_arm_bt
{

class ObjectVisible : public BT::ConditionNode
{
public:
  ObjectVisible(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<rclcpp::Node>& node)
    : BT::ConditionNode(name, config), node_(node)
  {
    client_ = node_->create_client<so_arm_bt_interfaces::srv::DetectObjects>("detect_objects");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("object_id"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("found_pose"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("hover_pose") };
  }


  BT::NodeStatus tick() override
  {
    std::string object_id;
    if (!getInput("object_id", object_id)) return BT::NodeStatus::FAILURE;

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(node_->get_logger(), "Vision service not available yet...");
      return BT::NodeStatus::FAILURE;
    }


    auto request = std::make_shared<so_arm_bt_interfaces::srv::DetectObjects::Request>();
    request->category_filter = object_id; 

    auto result_future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::milliseconds(500)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result_future.get();
      if (response->success && !response->detections.empty()) {
          auto pick_pose = response->detections[0].pose;
          setOutput("found_pose", pick_pose);
          
          // Calculate hover pose (e.g., 10cm above pick pose)
          auto hover_pose = pick_pose;
          hover_pose.pose.position.z += 0.10; // 10cm offset
          setOutput("hover_pose", hover_pose);
          
          return BT::NodeStatus::SUCCESS;
      }
    }


    return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<so_arm_bt_interfaces::srv::DetectObjects>::SharedPtr client_;
};

} // namespace so_arm_bt

#endif // SO_ARM_BT_VISION_NODES_HPP
