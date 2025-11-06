// Copyright 2022 Chen Jun
// Copyright 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

enum class ArmorName {
    not_armor,
    one,
    base,
    two,
    sentry,
    outpost
    // 其他盔甲名称...
};

struct Light : public cv::Rect
{
  Light() = default;
  explicit Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
  : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle)
  {
    length = cv::norm(top - bottom);
    width = area / length;
    center = (top + bottom) / 2;
  }

  int color;
  cv::Point2f top, bottom;
  cv::Point2f center;
  double length;
  double width;
  float tilt_angle;
};

struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  
  // Additional member for normalized center (relative to image size)
  cv::Point2f center_norm;  // 归一化后的中心坐标 (相对于图像大小)

  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classification_result;

  // Additional fields for armor name and color
  ArmorName name = ArmorName::not_armor;  // 默认是 no_armor
  int color = RED;  // 默认红色

  // Additional members for storing the points of the armor (e.g., the corners)
  std::vector<cv::Point2f> points;  // 盔甲的关键点（例如四个角点）

  // Debugging information (if needed)
  std::string debug_info;  // 可选：用于存储调试信息
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_

