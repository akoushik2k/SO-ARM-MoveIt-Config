import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('so_arm_bt')
    
    default_bt_xml = os.path.join(pkg_share, 'behavior_trees', 'test_tree.xml')
    
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml,
        description='Path to the Behavior Tree XML file'
    )
    
    # We need to make sure the MoveIt parameters are loaded.
    # Usually, if we launch this separately, we might need to load robot_description/semantic again
    # OR rely on the fact that if we run in the same namespace (or if parameters are global/on a server) it might work.
    # However, MoveGroupInterface often expects to find `robot_description_semantic` parameter on its own node or the move_group node.
    # 
    # Since the user runs MoveIt separately, `move_group` node is running.
    # The `MoveGroupInterface` inside our node needs to know the robot description.
    # It usually fetches it from the `robot_description` topic or parameter server.
    #
    # Best practice: Load the SRDF/URDF and pass it to this node just in case.
    # But for Phase 1 "plumbing", let's assume the user launches `demo.launch.py` which puts everything on the param server?
    # Actually, `demo.launch.py` usually launches `move_group`, which has the params.
    # BUT, `MoveGroupInterface` constructs a `RobotModel` often by looking for `robot_description` parameter ON THE LOCAL NODE if it can't find it elsewhere?
    # No, it looks for the topic.
    
    # Let's try minimal launch first.
    
    bt_node = Node(
        package='so_arm_bt',
        executable='bt_executor_node',
        name='bt_executor_node',
        output='screen',
        parameters=[
            {'bt_xml': LaunchConfiguration('bt_xml')}
        ]
    )

    return LaunchDescription([
        bt_xml_arg,
        bt_node
    ])
