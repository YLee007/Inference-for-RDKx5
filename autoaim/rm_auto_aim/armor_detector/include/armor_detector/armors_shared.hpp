#ifndef ARMOR_DETECTOR__ARMORS_SHARED_HPP_
#define ARMOR_DETECTOR__ARMORS_SHARED_HPP_

#include <vector>
#include <string>
#include <cstdint>
#include <opencv2/core.hpp>

namespace rm_auto_aim {
struct ArmorDetection {
	std::vector<cv::Point2f> kpts;
	std::string class_name;
	float score = 0.0f;

	std::string frame_id;
	int64_t stamp_sec = 0;
	uint32_t stamp_nanosec = 0;
};

extern std::vector<ArmorDetection> armors_keypoints;
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMORS_SHARED_HPP_
