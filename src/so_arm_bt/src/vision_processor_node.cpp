#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "so_arm_bt/object_detector.hpp"
#include "so_arm_bt/grasp_pose_estimator.hpp"
#include "so_arm_bt_interfaces/srv/detect_objects.hpp"
#include "so_arm_bt_interfaces/srv/compute_grasp_pose.hpp"

namespace so_arm_bt
{

class VisionProcessorNode : public rclcpp::Node
{
public:
    VisionProcessorNode() : Node("vision_processor_node")
    {
        // Declare parameters
        this->declare_parameter("camera_topic", "/camera/color/image_raw");
        this->declare_parameter("depth_topic", "/camera/depth/image_raw");
        this->declare_parameter("camera_info_topic", "/camera/color/camera_info");
        this->declare_parameter("detection_model", "yolov8n.onnx");
        this->declare_parameter("confidence_threshold", 0.5);
        this->declare_parameter("nms_threshold", 0.4);
        this->declare_parameter("grasp_method", "geometric");
        this->declare_parameter("grasp_approach_distance", 0.10);
        this->declare_parameter("camera_frame", "camera_color_optical_frame");
        this->declare_parameter("base_frame", "base_link");

        // Get parameters
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
        std::string model_path = this->get_parameter("detection_model").as_string();
        float conf_thresh = this->get_parameter("confidence_threshold").as_double();
        float nms_thresh = this->get_parameter("nms_threshold").as_double();
        std::string grasp_method = this->get_parameter("grasp_method").as_string();
        double approach_dist = this->get_parameter("grasp_approach_distance").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();

        // Initialize detector and grasp estimator
        try {
            detector_ = std::make_unique<ObjectDetector>(model_path, conf_thresh, nms_thresh);
            RCLCPP_INFO(this->get_logger(), "Object detector initialized with model: %s", model_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize detector: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Running without object detection capability");
        }

        // Set grasp method
        GraspPoseEstimator::GraspMethod method = GraspPoseEstimator::GraspMethod::GEOMETRIC;
        if (grasp_method == "point_cloud") {
            method = GraspPoseEstimator::GraspMethod::POINT_CLOUD;
        }
        grasp_estimator_ = std::make_unique<GraspPoseEstimator>(method, approach_dist);

        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&VisionProcessorNode::imageCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, 10,
            std::bind(&VisionProcessorNode::depthCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&VisionProcessorNode::cameraInfoCallback, this, std::placeholders::_1));

        // Publishers
        detection_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
            "detected_objects", 10);

        // Services
        detect_service_ = this->create_service<so_arm_bt_interfaces::srv::DetectObjects>(
            "detect_objects",
            std::bind(&VisionProcessorNode::detectObjectsCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        grasp_service_ = this->create_service<so_arm_bt_interfaces::srv::ComputeGraspPose>(
            "compute_grasp_pose",
            std::bind(&VisionProcessorNode::computeGraspCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Vision processor node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to camera: %s", camera_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to depth: %s", depth_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to camera_info: %s", camera_info_topic.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_image_ = msg;
        RCLCPP_DEBUG(this->get_logger(), "Received image: %dx%d", msg->width, msg->height);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_depth_ = msg;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_) {
            camera_info_ = msg;
            // Extract camera intrinsics
            camera_intrinsics_ << msg->k[0], msg->k[1], msg->k[2],
                                  msg->k[3], msg->k[4], msg->k[5],
                                  msg->k[6], msg->k[7], msg->k[8];
            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera info received");
        }
    }

    void detectObjectsCallback(
        const std::shared_ptr<so_arm_bt_interfaces::srv::DetectObjects::Request> request,
        std::shared_ptr<so_arm_bt_interfaces::srv::DetectObjects::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Detection service called with class='%s', min_conf=%.2f", 
                   request->object_class.c_str(), request->min_confidence);
        
        if (!latest_image_) {
            response->success = false;
            response->message = "No image available";
            RCLCPP_WARN(this->get_logger(), "No image available - check camera topics!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Image available: %dx%d, encoding=%s", 
                   latest_image_->width, latest_image_->height, latest_image_->encoding.c_str());

        if (!detector_) {
            response->success = false;
            response->message = "Detector not initialized";
            return;
        }

        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_image_, "bgr8");
            
            // Run detection
            auto detections = detector_->detect(cv_ptr->image);
            
            RCLCPP_INFO(this->get_logger(), "Detector found %zu raw detections", detections.size());

            // Filter by class and confidence
            vision_msgs::msg::Detection3DArray detection_array;
            detection_array.header = latest_image_->header;

            for (const auto& det : detections) {
                RCLCPP_INFO(this->get_logger(), "  - %s: conf=%.2f, bbox=[%d,%d,%d,%d]",
                           det.class_name.c_str(), det.confidence,
                           det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
                
                if (det.confidence < request->min_confidence) {
                    RCLCPP_DEBUG(this->get_logger(), "    Filtered by confidence");
                    continue;
                }
                if (!request->object_class.empty() && det.class_name != request->object_class) {
                    RCLCPP_DEBUG(this->get_logger(), "    Filtered by class");
                    continue;
                }

                vision_msgs::msg::Detection3D detection_3d;
                detection_3d.header = latest_image_->header;
                
                // Compute 3D position and grasp pose from depth
                if (latest_depth_ && camera_info_received_ && grasp_estimator_) {
                    try {
                        // Convert depth image to OpenCV
                        cv_bridge::CvImagePtr depth_ptr;
                        if (latest_depth_->encoding == "32FC1") {
                            depth_ptr = cv_bridge::toCvCopy(latest_depth_, "32FC1");
                        } else if (latest_depth_->encoding == "16UC1") {
                            depth_ptr = cv_bridge::toCvCopy(latest_depth_, "16UC1");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Unsupported depth encoding: %s", 
                                       latest_depth_->encoding.c_str());
                            depth_ptr = cv_bridge::toCvCopy(latest_depth_);
                        }
                        
                        // Compute grasp pose
                        auto grasp = grasp_estimator_->computeGrasp(
                            det.bbox,
                            depth_ptr->image,
                            camera_intrinsics_,
                            latest_image_->header.frame_id
                        );
                        
                        // Populate 3D bounding box with grasp pose
                        detection_3d.bbox.center = grasp.pose.pose;
                        
                        // Store detection result with class and confidence
                        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
                        hypothesis.hypothesis.class_id = std::to_string(det.class_id);
                        hypothesis.hypothesis.score = det.confidence;
                        hypothesis.pose.pose = grasp.pose.pose;
                        detection_3d.results.push_back(hypothesis);
                        
                        // Set ID to class name
                        detection_3d.id = det.class_name;
                        
                        RCLCPP_INFO(this->get_logger(), 
                                   "    3D pose: [%.3f, %.3f, %.3f], quality: %.2f",
                                   grasp.pose.pose.position.x,
                                   grasp.pose.pose.position.y,
                                   grasp.pose.pose.position.z,
                                   grasp.quality_score);
                        
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), 
                                   "Failed to compute 3D pose: %s", e.what());
                    }
                } else {
                    if (!latest_depth_) {
                        RCLCPP_DEBUG(this->get_logger(), "No depth image available");
                    }
                    if (!camera_info_received_) {
                        RCLCPP_DEBUG(this->get_logger(), "Camera info not received");
                    }
                }
                
                detection_array.detections.push_back(detection_3d);
            }

            response->success = true;
            response->detections = detection_array;
            response->message = "Detected " + std::to_string(detection_array.detections.size()) + " objects";
            
            RCLCPP_INFO(this->get_logger(), "Returning %zu filtered detections", 
                       detection_array.detections.size());

            // Publish detections
            detection_pub_->publish(detection_array);

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Detection failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void computeGraspCallback(
        const std::shared_ptr<so_arm_bt_interfaces::srv::ComputeGraspPose::Request> request,
        std::shared_ptr<so_arm_bt_interfaces::srv::ComputeGraspPose::Response> response)
    {
        if (!latest_depth_ || !camera_info_received_) {
            response->success = false;
            response->message = "Depth image or camera info not available";
            return;
        }

        try {
            // Convert depth image to OpenCV
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(latest_depth_);
            
            // TODO: Extract object region from detection
            // For now, use a simple approach: compute grasp from object pose
            
            // Compute grasp pose
            // This is a placeholder - needs actual implementation
            
            response->success = true;
            response->grasp_pose = request->object_pose;
            response->pre_grasp_pose = request->object_pose;
            response->quality_score = 0.8f;
            response->message = "Grasp computed (placeholder)";

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Grasp computation failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // Members
    std::unique_ptr<ObjectDetector> detector_;
    std::unique_ptr<GraspPoseEstimator> grasp_estimator_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
    rclcpp::Service<so_arm_bt_interfaces::srv::DetectObjects>::SharedPtr detect_service_;
    rclcpp::Service<so_arm_bt_interfaces::srv::ComputeGraspPose>::SharedPtr grasp_service_;

    sensor_msgs::msg::Image::SharedPtr latest_image_;
    sensor_msgs::msg::Image::SharedPtr latest_depth_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    Eigen::Matrix3d camera_intrinsics_;
    bool camera_info_received_ = false;

    std::string camera_frame_;
    std::string base_frame_;
};

} // namespace so_arm_bt

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<so_arm_bt::VisionProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
