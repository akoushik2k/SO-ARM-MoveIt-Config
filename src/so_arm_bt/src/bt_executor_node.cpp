#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "so_arm_bt/move_to_pose_action.hpp"
#include "so_arm_bt/gripper_moveit_actions.hpp"
#include "so_arm_bt/check_planning_scene.hpp"
#include "so_arm_bt/detect_objects_action.hpp"

#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("bt_executor_node");

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_executor_node");

    // Params
    node->declare_parameter("bt_xml", "");
    std::string bt_xml;
    if (!node->get_parameter("bt_xml", bt_xml) || bt_xml.empty()) {
        RCLCPP_ERROR(LOGGER, "Parameter 'bt_xml' is empty");
        return 1;
    }

    // MoveIt Interface
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
    auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");

    // Create executor for spinning
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);

    BT::BehaviorTreeFactory factory;

    // Register Nodes
    factory.registerBuilder<so_arm_bt::MoveToPoseSync>("MoveToPose",
        [move_group](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<so_arm_bt::MoveToPoseSync>(name, config, move_group);
        });

    factory.registerBuilder<so_arm_bt::OpenGripperMoveIt>("OpenGripper",
        [gripper_group](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<so_arm_bt::OpenGripperMoveIt>(name, config, gripper_group);
        });
    
    factory.registerBuilder<so_arm_bt::CloseGripperMoveIt>("CloseGripper",
        [gripper_group](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<so_arm_bt::CloseGripperMoveIt>(name, config, gripper_group);
        });

    factory.registerNodeType<so_arm_bt::CheckPlanningScene>("CheckPlanningScene");
    
    // Register vision nodes
    factory.registerBuilder<so_arm_bt::DetectObjectsAction>("DetectObjects",
        [node, executor](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<so_arm_bt::DetectObjectsAction>(name, config, node, executor);
        });


    // Load Tree
    auto tree = factory.createTreeFromFile(bt_xml);

    // Spin in main loop
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        BT::NodeStatus status = tree.tickOnce();
        
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
            RCLCPP_INFO(LOGGER, "BT Finished with status: %s", toStr(status).c_str());
            break; 
        }

        executor->spin_some();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
