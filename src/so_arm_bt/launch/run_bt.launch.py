import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("so101_new_calib", package_name="so_arm_moveit_config").to_moveit_configs()
    
    # Default to the generated_tree.xml in the package share directory
    default_tree_file = os.path.join(get_package_share_directory('so_arm_bt'), 'config', 'generated_tree.xml')

    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value=default_tree_file,
        description='Path to the behavior tree XML file'
    )

    return LaunchDescription([
        tree_file_arg,
        
        # BT Executor Node (Only)
        Node(
            package='so_arm_bt',
            executable='bt_executor_node',
            name='bt_executor_node',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {
                    'tree_file': LaunchConfiguration('tree_file')
                }
            ]
        )
    ])
