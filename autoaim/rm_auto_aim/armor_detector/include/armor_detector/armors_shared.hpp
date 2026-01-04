#ifndef ARMOR_DETECTOR__ARMORS_SHARED_HPP_
#define ARMOR_DETECTOR__ARMORS_SHARED_HPP_

#include <vector>
#include <string>
#include <cstdint>
#include <functional>
#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

namespace rm_auto_aim {
struct ArmorDetection {
	std::vector<cv::Point2f> kpts;
	std::string class_name;
	float score = 0.0f;

	std::string frame_id;
	int64_t stamp_sec = 0;
	uint32_t stamp_nanosec = 0;
};

using DetectionList = std::vector<ArmorDetection>;
struct DetectionBundle {
	DetectionList detections;
	cv::Mat image_bgr;
	std_msgs::msg::Header header;
};
using DetectionCallback = std::function<void(DetectionBundle&&)>;

// Register a callback to receive detections in-process (no global buffer).
void set_detection_callback(DetectionCallback cb);

// Emit detections (and optional image/header) to the registered callback; no-op if not set.
void emit_detections(DetectionBundle&& bundle);
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMORS_SHARED_HPP_
