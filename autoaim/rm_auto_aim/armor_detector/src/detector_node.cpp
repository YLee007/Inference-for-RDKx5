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

#include <image_transport/image_transport.hpp>
#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"
#include "armor_detector/armors_shared.hpp"
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

  // 根据相机的共享内存设置决定订阅哪种图像话题
  bool use_shared_memory = this->declare_parameter("use_shared_memory", false);
  
  if (use_shared_memory) {
    // 订阅共享内存图像
    hbmem_img_sub_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      "/hbmem_img", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::hbmemImageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribing to shared memory image: /hbmem_img");
  } else {
    // 订阅常规图像
    regular_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/hik_camera/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribing to regular image: /hik_camera/image_raw");
  }
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

void ArmorDetectorNode::hbmemImageCallback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr hbmem_msg)
{
  // 将共享内存消息转换为标准图像消息
  auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
  
  // 设置图像基本信息
  img_msg->header.stamp = hbmem_msg->time_stamp;
  img_msg->header.frame_id = "camera_optical_frame";
  img_msg->height = hbmem_msg->height;
  img_msg->width = hbmem_msg->width;
  img_msg->encoding = "nv12";  // 从共享内存消息的编码字段获取
  img_msg->is_bigendian = false;
  img_msg->step = hbmem_msg->step;
  
  // 复制图像数据
  img_msg->data.resize(hbmem_msg->data_size);
  std::copy(hbmem_msg->data.begin(), 
            hbmem_msg->data.begin() + hbmem_msg->data_size, 
            img_msg->data.begin());
  
  // 调用原来的图像处理函数
  imageCallback(img_msg);
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;
  auto armors = detectArmors(img_msg);

  if (pnp_solver_ != nullptr && is_aim_task_) {
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
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
  }
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Prefer DNN-produced shared keypoints if available
  std::vector<Armor> armors;
  if (!rm_auto_aim::armors_keypoints.empty()) {
    // Reserve to avoid repeated allocations (typical few elements)
    armors.reserve(rm_auto_aim::armors_keypoints.size());

    // Small helper to parse class name
    auto ParseArmorName = [](const std::string &s) -> ArmorName {
      if (s.empty()) return ArmorName::B1;
      char color = 0;
      int num = 0;
      for (size_t i = 0; i < s.size(); ++i) {
        char ch = std::toupper(static_cast<unsigned char>(s[i]));
        if (ch == 'B' || ch == 'R') {
          color = ch;
          // parse following digits
          std::string digits;
          for (size_t j = i + 1; j < s.size(); ++j) {
            if (std::isdigit(static_cast<unsigned char>(s[j]))) digits.push_back(s[j]);
            else break;
          }
          if (!digits.empty()) num = std::stoi(digits);
          break;
        }
      }
      if (color == 0) return ArmorName::B1;
      if (color == 'B') {
        switch (num) {
          case 1: return ArmorName::B1;
          case 2: return ArmorName::B2;
          case 3: return ArmorName::B3;
          case 4: return ArmorName::B4;
          case 5: return ArmorName::B5;
          case 7: return ArmorName::B7;
          default: return ArmorName::B1;
        }
      } else {
        switch (num) {
          case 1: return ArmorName::R1;
          case 2: return ArmorName::R2;
          case 3: return ArmorName::R3;
          case 4: return ArmorName::R4;
          case 5: return ArmorName::R5;
          case 7: return ArmorName::R7;
          default: return ArmorName::R1;
        }
      }
    };

    // Iterate by non-const reference so we can move keypoint vectors (avoid copies)
    for (auto & det : rm_auto_aim::armors_keypoints) {
      if (det.kpts.size() < 4) continue;

      // If detection contains a timestamp, ensure it matches the image timestamp (tolerance)
      if (det.stamp_sec != 0) {
        rclcpp::Time det_ts(rclcpp::Time(det.stamp_sec, det.stamp_nanosec));
        rclcpp::Time img_ts = img_msg->header.stamp;
        double dt = std::abs((img_ts - det_ts).seconds());
        if (dt > 0.2) { // 200 ms tolerance
          RCLCPP_DEBUG(this->get_logger(), "Skipping detection: timestamp mismatch (dt=%.3f s)", dt);
          continue;
        }
      }

      cv::Rect bbox = cv::boundingRect(det.kpts);
      cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);

      ArmorName parsed_name = ParseArmorName(det.class_name);
      // Mapping rule: B1 and R1 are LARGE, others are SMALL
      ArmorType atype = (parsed_name == ArmorName::B1 || parsed_name == ArmorName::R1) ? ArmorType::LARGE : ArmorType::SMALL;

      // Move keypoints into Armor to avoid copying vectors
      armors.emplace_back(parsed_name, det.score, bbox, std::move(det.kpts), center);
      Armor & armor = armors.back();
      armor.type = atype;
      // Keep full class string for UI (e.g. "B1", "R3")
      armor.classification_result = det.class_name;
      // For serial/communication we use team_id: 0 for blue (B), 1 for red (R), -1 unknown
      int team_id = -1;
      for (char c : det.class_name) {
        char cu = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        if (cu == 'B') { team_id = 0; break; }
        if (cu == 'R') { team_id = 1; break; }
      }
      armor.team_id = team_id;
    }
    // clear the shared buffer so next frame can write new detections
    rm_auto_aim::armors_keypoints.clear();
  }

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  if (debug_) {
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
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
