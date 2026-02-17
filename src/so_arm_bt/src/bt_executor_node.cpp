#include <rclcpp/rclcpp.hpp>
#include <filesystem>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "so_arm_bt/move_to_pose.hpp"
#include "so_arm_bt/gripper_actions.hpp"
#include "so_arm_bt/planning_scene_nodes.hpp"
#include "so_arm_bt/vision_nodes.hpp"
#include "so_arm_bt/precise_pick_node.hpp"
#include "so_arm_bt/condition_nodes.hpp"
#include "so_arm_bt/recovery_nodes.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_executor_node");

  // Allow time for things to start up
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Initialize MoveGroupInterfaces
  // Assuming 'arm' is the arm group and 'gripper' is the gripper group
  // Adjust these names based on your SRDF!
  auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
  auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");

  BT::BehaviorTreeFactory factory;

  // Register Custom Nodes
  factory.registerBuilder<so_arm_bt::MoveToPose>("MoveToPose",
                                                 [&](const std::string& name, const BT::NodeConfiguration& config)
                                                 {
                                                   return std::make_unique<so_arm_bt::MoveToPose>(name, config, arm_group);
                                                 });

  factory.registerBuilder<so_arm_bt::GripperAction>("GripperAction",
                                                    [&](const std::string& name, const BT::NodeConfiguration& config)
                                                    {
                                                      return std::make_unique<so_arm_bt::GripperAction>(name, config, gripper_group);
                                                    });

  factory.registerBuilder<so_arm_bt::AttachObject>("AttachObject", 
                                [&](const std::string& name, const BT::NodeConfiguration& config)
                                {
                                  return std::make_unique<so_arm_bt::AttachObject>(name, config);
                                });

  factory.registerBuilder<so_arm_bt::DetachObject>("DetachObject", 
                                [&](const std::string& name, const BT::NodeConfiguration& config)
                                {
                                  return std::make_unique<so_arm_bt::DetachObject>(name, config);
                                });


  factory.registerBuilder<so_arm_bt::ObjectVisible>("ObjectVisible",
                                                    [&](const std::string& name, const BT::NodeConfiguration& config)
                                                    {
                                                      return std::make_unique<so_arm_bt::ObjectVisible>(name, config, node);
                                                    });

  factory.registerBuilder<so_arm_bt::PrecisePick>("PrecisePick",
                                                    [&](const std::string& name, const BT::NodeConfiguration& config)
                                                    {
                                                      return std::make_unique<so_arm_bt::PrecisePick>(name, config, arm_group);
                                                    });

  factory.registerBuilder<so_arm_bt::AtPose>("AtPose",
                                             [&](const std::string& name, const BT::NodeConfiguration& config)
                                             {
                                               return std::make_unique<so_arm_bt::AtPose>(name, config, arm_group);
                                             });

  factory.registerBuilder<so_arm_bt::ClearOctomap>("ClearOctomap",
                                                   [&](const std::string& name, const BT::NodeConfiguration& config)
                                                   {
                                                     return std::make_unique<so_arm_bt::ClearOctomap>(name, config, node);
                                                   });

  factory.registerBuilder<so_arm_bt::GoHome>("GoHome",
                                             [&](const std::string& name, const BT::NodeConfiguration& config)
                                             {
                                               return std::make_unique<so_arm_bt::GoHome>(name, config, arm_group);
                                             });

  factory.registerSimpleAction("Wait", 
                                [&](BT::TreeNode& node) {
                                  int msec;
                                  if (!node.getInput("msec", msec)) return BT::NodeStatus::FAILURE;
                                  std::this_thread::sleep_for(std::chrono::milliseconds(msec));
                                  return BT::NodeStatus::SUCCESS;
                                }, { BT::InputPort<int>("msec") });



  // Get tree file path from parameter
  node->declare_parameter("tree_file", "");
  std::string tree_file = node->get_parameter("tree_file").as_string();

  if (tree_file.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No tree_file parameter provided");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Loading tree from: %s", tree_file.c_str());
  
  // Check if file exists to avoid realpath crash
  if (!std::filesystem::exists(tree_file))
  {
    RCLCPP_ERROR(node->get_logger(), "Tree file does not exist: %s. Please provide an absolute path or ensure it's in the current directory.", tree_file.c_str());
    return 1;
  }

  BT::Tree tree;
  try 
  {
    tree = factory.createTreeFromFile(tree_file);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to create tree from XML: %s", e.what());
    return 1;
  }


  // Logger
  BT::StdCoutLogger logger_cout(tree);

  // Tick the tree
  RCLCPP_INFO(node->get_logger(), "Starting BT execution...");
  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  
  // Simple loop
  rclcpp::Rate rate(10);
  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      status = tree.tickRoot();
      rclcpp::spin_some(node); // Important to handle ROS callbacks if we have any (MoveIt uses its own async stuff usually)
      rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "BT Execution finished with status: %s", toStr(status, true).c_str());

  rclcpp::shutdown();
  return 0;
}
