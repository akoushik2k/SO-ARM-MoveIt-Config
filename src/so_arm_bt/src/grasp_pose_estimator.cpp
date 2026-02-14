#include "so_arm_bt/grasp_pose_estimator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace so_arm_bt
{

GraspPoseEstimator::GraspPoseEstimator(
    GraspMethod method,
    double approach_distance)
    : method_(method)
    , approach_distance_(approach_distance)
{
    RCLCPP_INFO(rclcpp::get_logger("GraspPoseEstimator"),
                "GraspPoseEstimator initialized with method: %d, approach distance: %.3f",
                static_cast<int>(method), approach_distance);
}

GraspPose GraspPoseEstimator::computeGrasp(
    const cv::Rect& bbox,
    const cv::Mat& depth_image,
    const Eigen::Matrix3d& camera_intrinsics,
    const std::string& camera_frame)
{
    GraspPose grasp_pose;
    
    if (depth_image.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("GraspPoseEstimator"),
                    "Empty depth image provided");
        grasp_pose.quality_score = 0.0f;
        return grasp_pose;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("GraspPoseEstimator"),
                 "Computing grasp for bbox [%d, %d, %d, %d]",
                 bbox.x, bbox.y, bbox.width, bbox.height);
    
    // Compute object 3D center and size from depth
    cv::Point3f object_center;
    cv::Vec3f object_size;
    computeObjectGeometry(bbox, depth_image, camera_intrinsics, object_center, object_size);
    
    RCLCPP_INFO(rclcpp::get_logger("GraspPoseEstimator"),
                "Object center: [%.3f, %.3f, %.3f], size: [%.3f, %.3f, %.3f]",
                object_center.x, object_center.y, object_center.z,
                object_size[0], object_size[1], object_size[2]);
    
    // Generate geometric grasp
    grasp_pose = computeGeometricGrasp(object_center, object_size, camera_frame);
    
    return grasp_pose;
}

GraspPose GraspPoseEstimator::computeGraspFromPointCloud(
    const sensor_msgs::msg::PointCloud2& point_cloud,
    const std::string& camera_frame)
{
    GraspPose grasp_pose;
    
    // TODO: Implement point cloud-based grasp computation
    RCLCPP_WARN(rclcpp::get_logger("GraspPoseEstimator"),
                "Point cloud grasp computation not yet implemented (stub)");
    
    // Return placeholder
    grasp_pose.pose.header.frame_id = camera_frame;
    grasp_pose.quality_score = 0.5f;
    grasp_pose.approach_direction = "top";
    
    return grasp_pose;
}

GraspPose GraspPoseEstimator::computeGeometricGrasp(
    const cv::Point3f& object_center,
    const cv::Vec3f& object_size,
    const std::string& camera_frame)
{
    GraspPose grasp_pose;
    
    // Create grasp pose (top-down approach for small objects)
    grasp_pose.pose.header.frame_id = camera_frame;
    grasp_pose.pose.header.stamp = rclcpp::Clock().now();
    
    // Position: at object center
    grasp_pose.pose.pose.position.x = object_center.x;
    grasp_pose.pose.pose.position.y = object_center.y;
    grasp_pose.pose.pose.position.z = object_center.z;
    
    // Determine grasp orientation based on object size
    // For small objects (cube): top-down grasp
    // For larger objects (crate): could use side grasp
    bool use_top_down = (object_size[0] < 0.15 && object_size[1] < 0.15);
    
    tf2::Quaternion quat;
    Eigen::Vector3d approach_direction;
    
    if (use_top_down) {
        // Top-down grasp: gripper pointing down
        quat.setRPY(0.0, M_PI / 2.0, 0.0);
        approach_direction = Eigen::Vector3d(0.0, 0.0, 1.0);  // Approach from above
        grasp_pose.approach_direction = "top";
        
        RCLCPP_INFO(rclcpp::get_logger("GraspPoseEstimator"),
                   "Using top-down grasp for small object");
    } else {
        // Side grasp: gripper horizontal
        quat.setRPY(0.0, 0.0, 0.0);
        approach_direction = Eigen::Vector3d(1.0, 0.0, 0.0);  // Approach from front
        grasp_pose.approach_direction = "side";
        
        RCLCPP_INFO(rclcpp::get_logger("GraspPoseEstimator"),
                   "Using side grasp for larger object");
    }
    
    grasp_pose.pose.pose.orientation.x = quat.x();
    grasp_pose.pose.pose.orientation.y = quat.y();
    grasp_pose.pose.pose.orientation.z = quat.z();
    grasp_pose.pose.pose.orientation.w = quat.w();
    
    // Create pre-grasp pose (offset along approach direction)
    grasp_pose.pre_grasp_pose = createPreGraspPose(grasp_pose.pose, approach_direction);
    
    // Quality score based on depth validity and object size
    float depth_quality = (object_center.z > 0.1 && object_center.z < 2.0) ? 1.0f : 0.5f;
    float size_quality = (object_size[0] > 0.02 && object_size[0] < 0.3) ? 1.0f : 0.7f;
    grasp_pose.quality_score = depth_quality * size_quality * 0.85f;
    
    RCLCPP_INFO(rclcpp::get_logger("GraspPoseEstimator"),
               "Grasp quality: %.2f, approach: %s",
               grasp_pose.quality_score, grasp_pose.approach_direction.c_str());
    
    return grasp_pose;
}

cv::Point3f GraspPoseEstimator::deproject(
    const cv::Point2f& pixel,
    float depth,
    const Eigen::Matrix3d& intrinsics)
{
    // Deproject pixel to 3D point using camera intrinsics
    float fx = intrinsics(0, 0);
    float fy = intrinsics(1, 1);
    float cx = intrinsics(0, 2);
    float cy = intrinsics(1, 2);
    
    float x = (pixel.x - cx) * depth / fx;
    float y = (pixel.y - cy) * depth / fy;
    float z = depth;
    
    return cv::Point3f(x, y, z);
}

void GraspPoseEstimator::computeObjectGeometry(
    const cv::Rect& bbox,
    const cv::Mat& depth_image,
    const Eigen::Matrix3d& intrinsics,
    cv::Point3f& center,
    cv::Vec3f& size)
{
    // Extract depth values within bounding box
    cv::Rect safe_bbox = bbox & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
    
    if (safe_bbox.area() == 0) {
        RCLCPP_WARN(rclcpp::get_logger("GraspPoseEstimator"),
                   "Bounding box outside image bounds");
        center = cv::Point3f(0.3f, 0.0f, 0.2f);
        size = cv::Vec3f(0.05f, 0.05f, 0.05f);
        return;
    }
    
    cv::Mat depth_roi = depth_image(safe_bbox);
    
    // Convert depth to meters if needed (check type)
    cv::Mat depth_meters;
    if (depth_image.type() == CV_32FC1) {
        depth_meters = depth_roi.clone();
    } else if (depth_image.type() == CV_16UC1) {
        depth_roi.convertTo(depth_meters, CV_32FC1, 0.001);  // mm to meters
    } else {
        RCLCPP_WARN(rclcpp::get_logger("GraspPoseEstimator"),
                   "Unexpected depth image type: %d", depth_image.type());
        depth_meters = depth_roi.clone();
    }
    
    // Filter out invalid depths (0 or NaN)
    std::vector<float> valid_depths;
    for (int y = 0; y < depth_meters.rows; ++y) {
        for (int x = 0; x < depth_meters.cols; ++x) {
            float d = depth_meters.at<float>(y, x);
            if (d > 0.01 && d < 5.0 && !std::isnan(d)) {
                valid_depths.push_back(d);
            }
        }
    }
    
    if (valid_depths.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("GraspPoseEstimator"),
                   "No valid depth values in bbox, using default");
        center = cv::Point3f(0.3f, 0.0f, 0.2f);
        size = cv::Vec3f(0.05f, 0.05f, 0.05f);
        return;
    }
    
    // Compute median depth (more robust than mean)
    std::sort(valid_depths.begin(), valid_depths.end());
    float median_depth = valid_depths[valid_depths.size() / 2];
    
    // Deproject bbox center to 3D
    cv::Point2f bbox_center(
        safe_bbox.x + safe_bbox.width / 2.0f,
        safe_bbox.y + safe_bbox.height / 2.0f
    );
    
    center = deproject(bbox_center, median_depth, intrinsics);
    
    // Estimate object size from bbox and depth
    // Deproject corners to estimate 3D size
    cv::Point2f top_left(safe_bbox.x, safe_bbox.y);
    cv::Point2f bottom_right(safe_bbox.x + safe_bbox.width, safe_bbox.y + safe_bbox.height);
    
    cv::Point3f tl_3d = deproject(top_left, median_depth, intrinsics);
    cv::Point3f br_3d = deproject(bottom_right, median_depth, intrinsics);
    
    float width = std::abs(br_3d.x - tl_3d.x);
    float height = std::abs(br_3d.y - tl_3d.y);
    
    // Depth range as proxy for object thickness
    float depth_range = valid_depths.back() - valid_depths.front();
    float thickness = std::max(0.02f, std::min(0.15f, depth_range));
    
    size = cv::Vec3f(width, height, thickness);
    
    RCLCPP_DEBUG(rclcpp::get_logger("GraspPoseEstimator"),
                "Computed geometry from %zu valid depth points, median depth: %.3f m",
                valid_depths.size(), median_depth);
}

geometry_msgs::msg::PoseStamped GraspPoseEstimator::createPreGraspPose(
    const geometry_msgs::msg::PoseStamped& grasp_pose,
    const Eigen::Vector3d& approach_direction)
{
    geometry_msgs::msg::PoseStamped pre_grasp = grasp_pose;
    
    // Offset along approach direction
    pre_grasp.pose.position.x += approach_direction.x() * approach_distance_;
    pre_grasp.pose.position.y += approach_direction.y() * approach_distance_;
    pre_grasp.pose.position.z += approach_direction.z() * approach_distance_;
    
    return pre_grasp;
}

} // namespace so_arm_bt
