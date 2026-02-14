#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace so_arm_bt
{

struct GraspPose
{
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::PoseStamped pre_grasp_pose;
    float quality_score;
    std::string approach_direction;  // "top", "side", etc.
};

/**
 * @brief Grasp pose estimator for robotic manipulation
 * 
 * Computes 6-DOF grasp poses from object detections and depth data.
 * Supports geometric heuristics and optional learning-based methods.
 */
class GraspPoseEstimator
{
public:
    enum class GraspMethod
    {
        GEOMETRIC,      // Simple top-down or geometric grasp
        POINT_CLOUD,    // Point cloud-based grasp
        LEARNED         // Neural network-based (future)
    };

    /**
     * @brief Constructor
     * @param method Grasp estimation method
     * @param approach_distance Distance for pre-grasp pose (meters)
     */
    GraspPoseEstimator(
        GraspMethod method = GraspMethod::GEOMETRIC,
        double approach_distance = 0.10);

    /**
     * @brief Compute grasp pose from object bounding box and depth
     * @param bbox Object bounding box in image coordinates
     * @param depth_image Depth image (CV_32FC1 or CV_16UC1)
     * @param camera_info Camera intrinsic parameters
     * @param camera_frame Frame ID for the camera
     * @return Computed grasp pose
     */
    GraspPose computeGrasp(
        const cv::Rect& bbox,
        const cv::Mat& depth_image,
        const Eigen::Matrix3d& camera_intrinsics,
        const std::string& camera_frame);

    /**
     * @brief Compute grasp pose from point cloud
     * @param point_cloud Object point cloud
     * @param camera_frame Frame ID for the camera
     * @return Computed grasp pose
     */
    GraspPose computeGraspFromPointCloud(
        const sensor_msgs::msg::PointCloud2& point_cloud,
        const std::string& camera_frame);

    /**
     * @brief Set approach distance for pre-grasp pose
     */
    void setApproachDistance(double distance) { approach_distance_ = distance; }

    /**
     * @brief Set grasp method
     */
    void setGraspMethod(GraspMethod method) { method_ = method; }

private:
    GraspMethod method_;
    double approach_distance_;  // Distance to retract for pre-grasp (meters)

    /**
     * @brief Compute geometric top-down grasp
     */
    GraspPose computeGeometricGrasp(
        const cv::Point3f& object_center,
        const cv::Vec3f& object_size,
        const std::string& camera_frame);

    /**
     * @brief Get 3D point from depth image
     */
    cv::Point3f deproject(
        const cv::Point2f& pixel,
        float depth,
        const Eigen::Matrix3d& intrinsics);

    /**
     * @brief Compute object center and size from depth region
     */
    void computeObjectGeometry(
        const cv::Rect& bbox,
        const cv::Mat& depth_image,
        const Eigen::Matrix3d& intrinsics,
        cv::Point3f& center,
        cv::Vec3f& size);

    /**
     * @brief Create pre-grasp pose by offsetting along approach direction
     */
    geometry_msgs::msg::PoseStamped createPreGraspPose(
        const geometry_msgs::msg::PoseStamped& grasp_pose,
        const Eigen::Vector3d& approach_direction);
};

} // namespace so_arm_bt
