// Copyright 2022 Chen Jun
// Copyright 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>
#include <vector>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

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
  // 基本标识
  std::string number;        // 兼容旧字段，不强制使用

  // 几何属性
  cv::Point2f center;        // 由关键点/包围框计算的图像中心
  cv::Point2f center_norm;   // 归一化中心（可选，若未使用可忽略）
  ArmorType type;            // SMALL / LARGE
  cv::Rect bbox;             // 由关键点计算的外接矩形
  std::vector<cv::Point2f> armor_keypoints; // PnP 期望顺序：BL, TL, TR, BR
  cv::Point2f offset_;       

  // 分类/置信
  float score;                       // 后处理的最终分数 obj×cls
  std::string classification_result; // 完整类别字符串（如 "B3"、"R5" 或 "G/Bs/Bb"）
  int team_id = -1;                  // 0 = blue, 1 = red, -1 = unknown

  // 可选：旧 Light 方案兼容（不强制填充）
  Light left_light, right_light;
  cv::Mat number_img;

    Armor() = default;
    Armor(float score,
      const cv::Rect & bbox,
      std::vector<cv::Point2f> armor_keypoints,
      const cv::Point2f & center)
    : number(),
    center(center),
    center_norm(),
    type(ArmorType::INVALID),
    bbox(bbox),
    armor_keypoints(std::move(armor_keypoints)),
    offset_(),
    score(score),
    classification_result(),
    team_id(-1)
  {}
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_