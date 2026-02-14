#!/usr/bin/env python3
"""
Publish static transform from robot base to camera frame.

This script publishes the transform between the robot's base frame
and the Isaac Sim camera frame based on the camera's position in the simulation.

Usage:
    ros2 run so_arm_bt camera_tf_publisher.py
"""

import rclcpp
from rclcpp.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class CameraTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher')
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Declare parameters for camera pose
        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.5)
        self.declare_parameter('camera_roll', 0.0)
        self.declare_parameter('camera_pitch', 0.0)
        self.declare_parameter('camera_yaw', 0.0)
        self.declare_parameter('parent_frame', 'base')
        self.declare_parameter('camera_frame', 'sim_camera')
        
        # Get parameters
        x = self.get_parameter('camera_x').value
        y = self.get_parameter('camera_y').value
        z = self.get_parameter('camera_z').value
        roll = self.get_parameter('camera_roll').value
        pitch = self.get_parameter('camera_pitch').value
        yaw = self.get_parameter('camera_yaw').value
        parent_frame = self.get_parameter('parent_frame').value
        camera_frame = self.get_parameter('camera_frame').value
        
        # Create transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = camera_frame
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Convert RPY to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        # Publish static transform
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f'Publishing static transform: {parent_frame} -> {camera_frame}')
        self.get_logger().info(f'  Position: [{x:.3f}, {y:.3f}, {z:.3f}]')
        self.get_logger().info(f'  Rotation (RPY): [{roll:.3f}, {pitch:.3f}, {yaw:.3f}]')

def main(args=None):
    rclcpp.init(args=args)
    node = CameraTFPublisher()
    
    try:
        rclcpp.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclcpp.shutdown()

if __name__ == '__main__':
    main()
