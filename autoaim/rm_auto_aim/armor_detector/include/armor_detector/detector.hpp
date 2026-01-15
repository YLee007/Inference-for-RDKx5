// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/yolo.hpp"

namespace rm_auto_aim
{
class Detector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    // area condition
    double min_fill_ratio;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  struct YoloParams
  {
    std::string model_path;
    float score_threshold;
    float nms_threshold;
    int nms_top_k;
  };

  Detector(
    const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a,
    const YoloParams & yolo);

  std::vector<Armor> detect(const cv::Mat & input);

  // Legacy traditional pipeline functions kept in source for reference.
  // They are no longer used after switching to YOLO.
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // For debug usage
  void drawResults(cv::Mat & img);

  int binary_thres;
  int detect_color;
  LightParams l;
  ArmorParams a;

  // Debug image kept for API compatibility, not used in YOLO mode.
  cv::Mat binary_img;

private:
  bool isLight(const Light & possible_light);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;

  std::unique_ptr<Yolo> yolo_;
  Yolo::Params yolo_params_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
