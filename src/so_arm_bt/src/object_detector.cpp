#include "so_arm_bt/object_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <fstream>

namespace so_arm_bt
{

ObjectDetector::ObjectDetector(
    const std::string& model_path,
    float confidence_threshold,
    float nms_threshold)
    : confidence_threshold_(confidence_threshold)
    , nms_threshold_(nms_threshold)
    , input_size_(640, 640)
    , use_yolo_(false)
{
    RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                "Initializing ObjectDetector with model: %s", 
                model_path.c_str());
    
    // Try to load YOLO ONNX model
    if (!model_path.empty() && std::ifstream(model_path).good()) {
        try {
            net_ = cv::dnn::readNetFromONNX(model_path);
            
            if (net_.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("ObjectDetector"), 
                           "Failed to load ONNX model from: %s", model_path.c_str());
                use_yolo_ = false;
            } else {
                // Set backend and target
                net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                
                use_yolo_ = true;
                RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                           "Successfully loaded YOLOv8 model. Using YOLO detection.");
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("ObjectDetector"), 
                        "OpenCV exception loading model: %s", e.what());
            use_yolo_ = false;
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("ObjectDetector"), 
                   "Model file not found: %s. Using color-based detection fallback.", 
                   model_path.c_str());
        use_yolo_ = false;
    }
}

void ObjectDetector::setClassNames(const std::vector<std::string>& class_names)
{
    class_names_ = class_names;
}

std::string ObjectDetector::getClassName(int class_id) const
{
    if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
        return class_names_[class_id];
    }
    return "unknown";
}

std::vector<Detection> ObjectDetector::detect(const cv::Mat& image)
{
    std::vector<Detection> detections;
    
    if (image.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("ObjectDetector"), "Empty image received");
        return detections;
    }
    
    // Use YOLO if model is loaded, otherwise fallback to color detection
    if (use_yolo_) {
        detections = detectWithYOLO(image);
        RCLCPP_DEBUG(rclcpp::get_logger("ObjectDetector"), 
                    "YOLO detection found %zu objects", detections.size());
    } else {
        detections = detectByColor(image);
        RCLCPP_DEBUG(rclcpp::get_logger("ObjectDetector"), 
                    "Color-based detection found %zu objects", detections.size());
    }
    
    return detections;
}

std::vector<Detection> ObjectDetector::detectWithYOLO(const cv::Mat& image)
{
    std::vector<Detection> detections;
    
    try {
        // Preprocess image
        cv::Mat blob = preprocessImage(image);
        
        // Run inference
        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        
        // Post-process outputs
        detections = postprocess(outputs, image.size());
        
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ObjectDetector"), 
                    "YOLO detection failed: %s. Falling back to color detection.", e.what());
        // Fall back to color detection
        detections = detectByColor(image);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ObjectDetector"), 
                    "YOLO detection failed: %s. Falling back to color detection.", e.what());
        // Fall back to color detection
        detections = detectByColor(image);
    }
    
    return detections;
}

std::vector<Detection> ObjectDetector::detectByColor(const cv::Mat& image)
{
    std::vector<Detection> detections;
    
    // Convert to HSV for color detection
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    
    // Detect red cube
    cv::Mat red_mask1, red_mask2, red_mask;
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
    
    // Find contours for red objects
    std::vector<std::vector<cv::Point>> red_contours;
    cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : red_contours) {
        double area = cv::contourArea(contour);
        if (area > 100) {  // Lowered threshold to detect smaller cubes
            cv::Rect bbox = cv::boundingRect(contour);
            Detection det;
            det.class_id = 0;  // Custom ID for cube
            det.class_name = "cube";
            det.confidence = 0.9f;
            det.bbox = bbox;
            detections.push_back(det);
            
            RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                        "Detected red cube at [%d, %d, %d, %d], area=%.0f", 
                        bbox.x, bbox.y, bbox.width, bbox.height, area);
        }
    }
    
    // Detect purple/pink crate
    cv::Mat purple_mask;
    cv::inRange(hsv, cv::Scalar(130, 30, 30), cv::Scalar(170, 255, 255), purple_mask);
    
    std::vector<std::vector<cv::Point>> purple_contours;
    cv::findContours(purple_mask, purple_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : purple_contours) {
        double area = cv::contourArea(contour);
        if (area > 300) {  // Lowered threshold for smaller crates
            cv::Rect bbox = cv::boundingRect(contour);
            Detection det;
            det.class_id = 1;  // Custom ID for crate
            det.class_name = "crate";
            det.confidence = 0.85f;
            det.bbox = bbox;
            detections.push_back(det);
            
            RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                        "Detected crate at [%d, %d, %d, %d], area=%.0f", 
                        bbox.x, bbox.y, bbox.width, bbox.height, area);
        }
    }
    
    return detections;
}

cv::Mat ObjectDetector::preprocessImage(const cv::Mat& image)
{
    // Resize to model input size while maintaining aspect ratio
    cv::Mat resized;
    cv::resize(image, resized, input_size_);
    
    // Convert to blob (NCHW format, normalize to [0,1])
    cv::Mat blob = cv::dnn::blobFromImage(
        resized, 
        1.0 / 255.0,      // Scale factor
        input_size_,      // Size
        cv::Scalar(0, 0, 0),  // Mean subtraction (none for YOLOv8)
        true,             // swapRB (BGR to RGB)
        false,            // crop
        CV_32F            // ddepth
    );
    
    return blob;
}

std::vector<Detection> ObjectDetector::postprocess(
    const std::vector<cv::Mat>& outputs,
    const cv::Size& original_size)
{
    std::vector<Detection> detections;
    
    if (outputs.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("ObjectDetector"), "No outputs from YOLO");
        return detections;
    }
    
    cv::Mat output = outputs[0];
    
    // Debug: Print output shape
    RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
               "YOLO output dims=%d", output.dims);
    if (output.dims == 3) {
        RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                   "YOLO output shape: [%d, %d, %d]", 
                   output.size[0], output.size[1], output.size[2]);
    } else if (output.dims == 2) {
        RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                   "YOLO output shape: [%d, %d]", 
                   output.rows, output.cols);
    }
    
    // YOLOv8 output format: [1, 84, 8400] for COCO
    // 84 = 4 (bbox) + 80 (classes)
    // 8400 = number of predictions
    
    // Reshape to [8400, 84] if needed
    if (output.dims == 3 && output.size[0] == 1) {
        // Shape is [1, 84, 8400], need to transpose to [8400, 84]
        int num_classes_plus_4 = output.size[1];  // 84
        int num_predictions = output.size[2];      // 8400
        
        RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                   "Reshaping from [1, %d, %d] to [%d, %d]",
                   num_classes_plus_4, num_predictions, 
                   num_predictions, num_classes_plus_4);
        
        // Create a 2D matrix [84, 8400]
        cv::Mat output_2d(num_classes_plus_4, num_predictions, CV_32F, output.ptr<float>());
        
        // Transpose to [8400, 84]
        cv::transpose(output_2d, output);
    }
    
    int rows = output.rows;  // Number of detections (8400)
    int dimensions = output.cols;  // 84 for COCO (4 bbox + 80 classes)
    
    RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
               "Processing %d predictions with %d dimensions", rows, dimensions);
    
    // Scale factors for bbox coordinates
    float x_factor = static_cast<float>(original_size.width) / input_size_.width;
    float y_factor = static_cast<float>(original_size.height) / input_size_.height;
    
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    // Parse detections
    for (int i = 0; i < rows; ++i) {
        float* data = output.ptr<float>(i);
        
        // Get class scores (skip first 4 bbox values)
        cv::Mat scores(1, dimensions - 4, CV_32F, data + 4);
        cv::Point class_id_point;
        double max_class_score;
        cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);
        
        // Filter by confidence threshold
        if (max_class_score > confidence_threshold_) {
            // YOLOv8 bbox format: [center_x, center_y, width, height]
            float cx = data[0];
            float cy = data[1];
            float w = data[2];
            float h = data[3];
            
            // Convert to [x, y, width, height] and scale to original image
            int left = static_cast<int>((cx - w / 2.0f) * x_factor);
            int top = static_cast<int>((cy - h / 2.0f) * y_factor);
            int width = static_cast<int>(w * x_factor);
            int height = static_cast<int>(h * y_factor);
            
            // Clamp to image boundaries
            left = std::max(0, std::min(left, original_size.width - 1));
            top = std::max(0, std::min(top, original_size.height - 1));
            width = std::min(width, original_size.width - left);
            height = std::min(height, original_size.height - top);
            
            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(static_cast<float>(max_class_score));
            class_ids.push_back(class_id_point.x);
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
               "Found %zu detections before NMS", boxes.size());
    
    // Apply Non-Maximum Suppression
    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, nms_indices);
    
    RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
               "Found %zu detections after NMS", nms_indices.size());
    
    // Create final detections
    for (int idx : nms_indices) {
        Detection det;
        det.class_id = class_ids[idx];
        det.class_name = getClassName(class_ids[idx]);
        det.confidence = confidences[idx];
        det.bbox = boxes[idx];
        detections.push_back(det);
        
        RCLCPP_INFO(rclcpp::get_logger("ObjectDetector"), 
                    "Detected %s (conf: %.2f) at [%d, %d, %d, %d]",
                    det.class_name.c_str(), det.confidence,
                    det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
    }
    
    return detections;
}

} // namespace so_arm_bt
