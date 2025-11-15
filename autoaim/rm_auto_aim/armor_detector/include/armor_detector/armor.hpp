// Copyright 2022 Chen Jun
// Copyright 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID };
enum class ArmorName {
  B1,
  B2,
  B3,
  B4,
  B5,
  B7,
  R1,
  R2,
  R3,
  R4,
  R5,
  R7
};
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Armor
{
  ArmorName name;
  std::string number;

  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;

  ArmorType type;

  cv::Rect bbox;
  ArmorName class_id;
  std::vector<cv::Point2f> armor_keypoints;
  cv::Point2f offset_;

  float confidence;
  std::string classification_result;
  // team id for serial comms: 0 = blue, 1 = red, -1 = unknown
  int team_id = -1;

  Armor(ArmorName class_id, float confidence, const cv::Rect & bbox, std::vector<cv::Point2f> armor_keypoints, const cv::Point2f & center)
  : name(class_id), center(center), bbox(bbox), class_id(class_id), armor_keypoints(armor_keypoints),
    confidence(confidence) {}
};


}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_

