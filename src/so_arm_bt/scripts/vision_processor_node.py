#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from so_arm_bt_interfaces.srv import DetectObjects
from so_arm_bt_interfaces.msg import Detection

class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')
        
        # Parameters
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('camera_frame', 'Camera')
        self.declare_parameter('coordinate_scaling', 1.0) # Default 1.0
        
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.scaling = self.get_parameter('coordinate_scaling').get_parameter_value().double_value
        
        self.bridge = CvBridge()
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/isaac_rgb', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/isaac_depth', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera_info', self.info_callback, 10)
        
        # Service
        self.srv = self.create_service(DetectObjects, 'detect_objects', self.detect_objects_callback)
        
        self.latest_image = None
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.camera_info = None
        self.latest_detections = []
        
        # Color ranges (HSV) - Adjust if needed for simulated colors
        self.colors = {
            'red': [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [180, 255, 255])],
            'green': [([35, 100, 100], [85, 255, 255])],
            'blue': [([100, 100, 100], [130, 255, 255])]
        }
        
        self.get_logger().info('Vision processor node initialized (Color-based)')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image()
        except Exception as e:
            self.get_logger().error(f'Image callback failed: {e}')

    def depth_callback(self, msg):
        try:
            # Isaac Sim depth is often 32FC1 (meters)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Depth callback failed: {e}')

    def info_callback(self, msg):
        self.camera_info = msg

    def process_image(self):
        if self.latest_image is None or self.camera_info is None or self.latest_depth is None:
            return

        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        detections = []

        for color_name, ranges in self.colors.items():
            mask = None
            for (lower, upper) in ranges:
                l = np.array(lower, dtype="uint8")
                u = np.array(upper, dtype="uint8")
                m = cv2.inRange(hsv, l, u)
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100: # Filter noise
                    continue
                
                # Get center
                M = cv2.moments(cnt)
                if M["m00"] == 0: continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Get depth
                if cy >= self.latest_depth.shape[0] or cx >= self.latest_depth.shape[1]:
                    continue
                depth = self.latest_depth[cy, cx]
                if np.isnan(depth) or depth <= 0:
                    continue
                
                # De-project to 3D
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx_off = self.camera_info.k[2]
                cy_off = self.camera_info.k[5]
                
                z = float(depth) * self.scaling
                x = (cx - cx_off) * z / fx
                y = (cy - cy_off) * z / fy
                
                # Prepare camera pose (identity orientation)
                pose_cam = PoseStamped()
                pose_cam.header.frame_id = self.camera_frame
                # Use depth stamp for better sync if available
                pose_cam.header.stamp = self.latest_depth_stamp if self.latest_depth_stamp else self.get_clock().now().to_msg()
                pose_cam.pose.position.x = x
                pose_cam.pose.position.y = y
                pose_cam.pose.position.z = z
                pose_cam.pose.orientation.w = 1.0

                # Transform to base
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        pose_cam.header.frame_id,
                        rclpy.time.Time())
                    
                    # Scale transform translation if simulation is in different units (e.g. feet)
                    transform.transform.translation.x *= self.scaling
                    transform.transform.translation.y *= self.scaling
                    transform.transform.translation.z *= self.scaling
                    
                    pose_base = tf2_geometry_msgs.do_transform_pose(pose_cam.pose, transform)
                    # pose_base.position.z += 0.05 # Removed: BT now handles offsets
                    
                    # Set orientation to face "down" (roughly 180 deg around Y) in base frame

                    # [0, 1, 0, 0] is 180 deg around Y (quat: x=0, y=1, z=0, w=0)
                    pose_base.orientation.x = 0.0
                    pose_base.orientation.y = 1.0
                    pose_base.orientation.z = 0.0
                    pose_base.orientation.w = 0.0
                    
                    # self.get_logger().info(f'RAW depth: {z:.3f} | Transformed base coords: x={pose_base.position.x:.3f}, y={pose_base.position.y:.3f}, z={pose_base.position.z:.3f}')


                    
                    det = Detection()
                    det.object_id = f"{color_name}_cube"
                    det.label = f"{color_name}_cube"
                    det.pose = PoseStamped()
                    det.pose.header.frame_id = self.base_frame
                    det.pose.header.stamp = pose_cam.header.stamp
                    det.pose.pose = pose_base
                    det.confidence = 1.0
                    detections.append(det)


                except Exception as e:
                    # Throttled logging might be better here
                    pass
        
        self.latest_detections = detections
        if detections:
            labels = [d.label for d in detections]
            # self.get_logger().info(f'Detected: {", ".join(set(labels))}', once=True)

    def detect_objects_callback(self, request, response):
        category = request.category_filter
        if category:
            # Handle 'cube' as a general filter for red_cube, green_cube, etc.
            response.detections = [d for d in self.latest_detections if category in d.label]
        else:
            response.detections = self.latest_detections
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
