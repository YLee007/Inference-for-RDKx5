// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <string>
#include <cctype>
#include <vector>
#include <sstream>

#include <image_transport/image_transport.hpp>
#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"
#include <std_msgs/msg/header.hpp>
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Task subscriber
  is_aim_task_ = true;
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/task_mode", 10, std::bind(&ArmorDetectorNode::taskCallback, this, std::placeholders::_1));

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/hik_camera/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camer_info) {
      cam_center_ = cv::Point2f(camer_info->k[2], camer_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camer_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camer_info->k, camer_info->d);
      cam_info_sub_.reset();
    });

  // In-process detection handoff: receive detections directly from YoloNode, single callback chain
  set_detection_callback([this](DetectionBundle && bundle) {
    this->onDetections(std::move(bundle));
  });
}

void ArmorDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  using std::placeholders::_1;
  std::string task_mode = task_msg->data;
  if (task_mode == "aim") {
    is_aim_task_ = true;
  } else {
    is_aim_task_ = false;
  }
}

void ArmorDetectorNode::onDetections(DetectionBundle && bundle)
{
  if (!pnp_solver_ || !is_aim_task_) {
    return;
  }

  auto armors = convertDetections(std::move(bundle.detections));

  // Header: prefer bundle header; fallback to detection timestamp
  std_msgs::msg::Header hdr = bundle.header;
  if (hdr.stamp.sec == 0 && hdr.stamp.nanosec == 0 && !armors.empty()) {
    hdr.frame_id = armors.front().source_frame;
    hdr.stamp.sec = armors.front().stamp_sec;
    hdr.stamp.nanosec = armors.front().stamp_nanosec;
  }
  armors_msg_.header = armor_marker_.header = text_marker_.header = hdr;

  armors_msg_.armors.clear();
  armors_msg_.armors.reserve(armors.size());
  marker_array_.markers.clear();
  armor_marker_.id = 0;
  text_marker_.id = 0;

  auto_aim_interfaces::msg::Armor armor_msg;
  for (const auto & armor : armors) {
    cv::Mat rvec, tvec;
    bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
    if (success) {
      // Fill basic info
      armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
      armor_msg.number = armor.classification_result;

      // Fill pose
      armor_msg.pose.position.x = tvec.at<double>(0);
      armor_msg.pose.position.y = tvec.at<double>(1);
      armor_msg.pose.position.z = tvec.at<double>(2);
      // rvec to 3x3 rotation matrix
      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);
      // rotation matrix to quaternion
      tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));
      tf2::Quaternion tf2_q;
      tf2_rotation_matrix.getRotation(tf2_q);
      armor_msg.pose.orientation = tf2::toMsg(tf2_q);

      // Fill the distance to image center
      armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

      // Fill keypoints (reserve small fixed size to avoid reallocs)
      armor_msg.kpts.clear();
      armor_msg.kpts.reserve(armor.armor_keypoints.size());
      for (const auto & pt : armor.armor_keypoints) {
        geometry_msgs::msg::Point point;
        point.x = pt.x;
        point.y = pt.y;
        armor_msg.kpts.emplace_back(point);
      }

      // Fill the markers
      armor_marker_.id++;
      armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
      armor_marker_.pose = armor_msg.pose;
      text_marker_.id++;
      text_marker_.pose.position = armor_msg.pose.position;
      text_marker_.pose.position.y -= 0.1;
      text_marker_.text = armor.classification_result;
      armors_msg_.armors.emplace_back(armor_msg);
      marker_array_.markers.emplace_back(armor_marker_);
      marker_array_.markers.emplace_back(text_marker_);
    } else {
      RCLCPP_WARN(this->get_logger(), "PnP failed!");
    }
  }

  // Publishing detected armors
  armors_pub_->publish(armors_msg_);

  // Publishing marker
  publishMarkers();

  // Optional visualization overlay on original image
  if (debug_ && !bundle.image_bgr.empty()) {
    cv::Mat vis = bundle.image_bgr.clone();
    // Draw camera center if known
    if (cam_info_) {
      cv::circle(vis, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    }
    // Draw armors and labels
    for (const auto & armor : armors) {
      if (armor.armor_keypoints.size() >= 4) {
        std::vector<cv::Point> poly;
        poly.reserve(armor.armor_keypoints.size());
        for (const auto & p : armor.armor_keypoints) {
          poly.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));
        }
        cv::polylines(vis, poly, true, cv::Scalar(0, 255, 0), 2);
        std::ostringstream label;
        label << armor.classification_result << " " << std::fixed << std::setprecision(2)
              << armor.score;
        cv::putText(vis, label.str(), poly.front(), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 0), 2);
      }
    }

    // Latency if timestamp available
    if (hdr.stamp.sec != 0 || hdr.stamp.nanosec != 0) {
      rclcpp::Time det_ts(hdr.stamp);
      auto latency = (this->now() - det_ts).seconds() * 1000;
      std::stringstream latency_ss;
      latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
      cv::putText(vis, latency_ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 255, 0), 2);
    }

    result_img_pub_.publish(cv_bridge::CvImage(hdr, "bgr8", vis).toImageMsg());
  }
}

std::vector<Armor> ArmorDetectorNode::convertDetections(DetectionList && dets)
{
  std::vector<Armor> armors;
  if (dets.empty()) {
    return armors;
  }

  armors.reserve(dets.size());

  for (auto & det : dets) {
    if (det.kpts.size() < 4) continue;

    cv::Rect bbox = cv::boundingRect(det.kpts);
    cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);

    // 类型判定：Bb 为大装甲；数字类中含"1"(B1/R1)判为大装甲；其余判为小装甲
    ArmorType atype = ArmorType::SMALL;
    if (det.class_name == "Bb") {
      atype = ArmorType::LARGE;
    } else {
      bool has_one_digit = false;
      for (char c : det.class_name) {
        if (std::isdigit(static_cast<unsigned char>(c)) && c == '1') { has_one_digit = true; break; }
      }
      if (has_one_digit) atype = ArmorType::LARGE;
    }

    // Move keypoints into Armor to avoid copying vectors
    armors.emplace_back(det.score, bbox, std::move(det.kpts), center);
    Armor & armor = armors.back();
    armor.type = atype;
    armor.classification_result = det.class_name;  // e.g. "B1", "R3"
    armor.source_frame = det.frame_id;
    armor.stamp_sec = det.stamp_sec;
    armor.stamp_nanosec = det.stamp_nanosec;

    int team_id = -1;
    for (char c : det.class_name) {
      char cu = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      if (cu == 'B') { team_id = 0; break; }
      if (cu == 'R') { team_id = 1; break; }
    }
    armor.team_id = team_id;
  }

  // Latency debug: use detection stamp if present
  if (debug_ && !armors.empty()) {
    rclcpp::Time det_ts(armors.front().stamp_sec, armors.front().stamp_nanosec);
    auto final_time = this->now();
    auto latency = (final_time - det_ts).seconds() * 1000;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");
  }

  return armors;
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  armors_data_pub_.reset();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::createDebugPublishers()
{
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
  armors_data_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugArmors>(
    "/detector/debug_armors", rclcpp::SensorDataQoS());
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
