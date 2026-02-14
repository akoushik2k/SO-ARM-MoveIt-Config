#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

namespace so_arm_bt
{

struct Detection
{
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect bbox;  // Bounding box in image coordinates
    cv::Point3f center_3d;  // 3D center point (if depth available)
};

/**
 * @brief Object detector using YOLO (ONNX format)
 * 
 * This class wraps OpenCV DNN module to run YOLO object detection.
 * Supports YOLOv8 ONNX models.
 */
class ObjectDetector
{
public:
    /**
     * @brief Constructor
     * @param model_path Path to ONNX model file
     * @param confidence_threshold Minimum confidence for detections (0.0 - 1.0)
     * @param nms_threshold Non-maximum suppression threshold (0.0 - 1.0)
     */
    ObjectDetector(
        const std::string& model_path,
        float confidence_threshold = 0.5f,
        float nms_threshold = 0.4f);

    /**
     * @brief Detect objects in an image
     * @param image Input image (BGR format)
     * @return Vector of detections
     */
    std::vector<Detection> detect(const cv::Mat& image);

    /**
     * @brief Set class names for detection
     * @param class_names Vector of class names (index = class_id)
     */
    void setClassNames(const std::vector<std::string>& class_names);

    /**
     * @brief Get class name from class ID
     * @param class_id Class ID
     * @return Class name string
     */
    std::string getClassName(int class_id) const;

    /**
     * @brief Update confidence threshold
     */
    void setConfidenceThreshold(float threshold) { confidence_threshold_ = threshold; }

    /**
     * @brief Update NMS threshold
     */
    void setNMSThreshold(float threshold) { nms_threshold_ = threshold; }

private:
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    float confidence_threshold_;
    float nms_threshold_;
    cv::Size input_size_;  // Model input size (e.g., 640x640 for YOLOv8)
    bool use_yolo_;        // Flag to indicate if YOLO model is loaded

    /**
     * @brief Detect objects using YOLO model
     * @param image Input image (BGR format)
     * @return Vector of detections
     */
    std::vector<Detection> detectWithYOLO(const cv::Mat& image);

    /**
     * @brief Preprocess image for YOLO input
     */
    cv::Mat preprocessImage(const cv::Mat& image);

    /**
     * @brief Post-process YOLO output
     */
    std::vector<Detection> postprocess(
        const std::vector<cv::Mat>& outputs,
        const cv::Size& original_size);
    
    /**
     * @brief Color-based detection fallback for simple objects
     * @param image Input image (BGR format)
     * @return Vector of detections based on color
     */
    std::vector<Detection> detectByColor(const cv::Mat& image);
};

} // namespace so_arm_bt
