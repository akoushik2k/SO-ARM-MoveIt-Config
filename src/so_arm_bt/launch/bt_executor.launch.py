import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("so101_new_calib", package_name="so_arm_moveit_config").to_moveit_configs()
    
    # Point to the package share directory
    default_tree_file = os.path.join(get_package_share_directory('so_arm_bt'), 'config', 'generated_tree.xml')

    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value=default_tree_file,
        description='Path to the behavior tree XML file'
    )

    return LaunchDescription([
        tree_file_arg,
        
        # Vision Processor Node
        Node(
            package='so_arm_bt',
            executable='vision_processor_node.py',
            name='vision_processor_node',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {
                    'model_path': os.path.join(os.getcwd(), 'yolov8n.onnx'),
                    'base_frame': 'base',
                    'camera_frame': 'Camera',
                    'use_sim_time': True,
                    'coordinate_scaling': 0.3048
                }
            ]
        ),

        # BT Compiler Node
        Node(
            package='so_arm_bt',
            executable='bt_compiler_node.py',
            name='bt_compiler_node',
            output='screen',
            parameters=[{
                'output_path': 'generated_tree.xml',
                'use_sim_time': True
            }]
        ),

        # BT Executor Node
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
                    'tree_file': LaunchConfiguration('tree_file'),
                    'use_sim_time': True
                }
            ]
        )
    ])



