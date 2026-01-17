// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"


namespace rm_auto_aim
{
static std::string format_id(int color_idx, int cls_idx)
{
  if (color_idx != RED && color_idx != BLUE) {
    return std::string();
  }
  const auto & names = Yolo::classNames();
  if (cls_idx < 0 || cls_idx >= static_cast<int>(names.size())) {
    return std::string();
  }
  const std::string prefix = (color_idx == RED) ? "R_" : "B_";
  return prefix + names[cls_idx];
}

Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a,
  const YoloParams & yolo)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)
{
  yolo_params_.model_path = yolo.model_path;
  yolo_params_.score_threshold = yolo.score_threshold;
  yolo_params_.nms_threshold = yolo.nms_threshold;
  yolo_params_.nms_top_k = yolo.nms_top_k;
  yolo_params_.detect_color = detect_color;
  yolo_ = std::make_unique<Yolo>(yolo_params_);
}

std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  armors_.clear();
  lights_.clear();

  if (!yolo_) {
    return armors_;
  }

  // Keep filter color in sync with node parameter
  yolo_params_.detect_color = detect_color;

  auto dets = yolo_->infer(input);
  armors_.reserve(dets.size());

  for (const auto & det : dets) {
    // det.kpts: TL, BL, BR, TR
    const auto & tl = det.kpts[0];
    const auto & bl = det.kpts[1];
    const auto & br = det.kpts[2];
    const auto & tr = det.kpts[3];

    // Build two "lights" so PnP order stays: LB,LT,RT,RB
    const auto left_box = cv::boundingRect(std::vector<cv::Point2f>{tl, bl});
    const auto right_box = cv::boundingRect(std::vector<cv::Point2f>{tr, br});
    Light left_light(left_box, tl, bl, std::max(1, left_box.area()), 0.0f);
    Light right_light(right_box, tr, br, std::max(1, right_box.area()), 0.0f);

    Armor armor(left_light, right_light);

    const std::string id = format_id(det.color_idx, det.cls_idx);
    if (id.empty()) {
      continue;
    }

    armor.number = id;
    armor.classfication_result = id;
    armor.confidence = det.score;

    // Minimal rule: "1" and "Bb" are large, others are small
    const auto & names = Yolo::classNames();
    const std::string base = (det.cls_idx >= 0 && det.cls_idx < static_cast<int>(names.size())) ?
                               names[det.cls_idx] :
                               std::string();
    armor.type = (base == "1" || base == "Bb") ? ArmorType::LARGE : ArmorType::SMALL;

    armors_.emplace_back(std::move(armor));
  }

  return armors_;
}

Yolo::Timings Detector::lastYoloTimings() const
{
  if (yolo_) {
    return yolo_->lastTimings();
  }
  return Yolo::Timings{};
}

std::vector<Light> Detector::findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    auto b_rect = cv::boundingRect(contour);
    auto r_rect = cv::minAreaRect(contour);
    cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
    std::vector<cv::Point> mask_contour;
    for (const auto & p : contour) {
      mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
    }
    cv::fillPoly(mask, {mask_contour}, 255);
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    // points / rotated rect area
    bool is_fill_rotated_rect =
      points.size() / (r_rect.size.width * r_rect.size.height) > l.min_fill_ratio;
    cv::Vec4f return_param;
    cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2f top, bottom;
    double angle_k;
    if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
      top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
      bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
      angle_k = 0;
    } else {
      auto k = return_param[1] / return_param[0];
      auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
      top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
      bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
      angle_k = std::atan(k) / CV_PI * 180 - 90;
      if (angle_k > 90) {
        angle_k = 180 - angle_k;
      }
    }
    auto light = Light(b_rect, top, bottom, points.size(), angle_k);

    if (isLight(light) && is_fill_rotated_rect) {
      auto rect = light;
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rgb_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  return is_light;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  return type;
}

void Detector::drawResults(cv::Mat & img)
{
  for (const auto & armor : armors_) {
    const auto tl = armor.left_light.top;
    const auto bl = armor.left_light.bottom;
    const auto br = armor.right_light.bottom;
    const auto tr = armor.right_light.top;

    const std::vector<cv::Point> poly = {
      cv::Point(static_cast<int>(tl.x), static_cast<int>(tl.y)),
      cv::Point(static_cast<int>(bl.x), static_cast<int>(bl.y)),
      cv::Point(static_cast<int>(br.x), static_cast<int>(br.y)),
      cv::Point(static_cast<int>(tr.x), static_cast<int>(tr.y))};

    cv::polylines(img, poly, true, cv::Scalar(0, 255, 0), 2);

    cv::putText(
      img, armor.classfication_result, tl, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255),
      2);
  }
}

}  // namespace rm_auto_aim
