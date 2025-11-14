#include "armor_detector/yolo11_node.hpp"
#include "armor_detector/armors_shared.hpp"
#include <opencv2/core.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

#include "dnn_node/util/output_parser/detection/ptq_yolo8_output_parser.h"
#include "dnn_node/util/output_parser/perception_common.h"
#include "dnn_node/util/image_proc.h"
#include "hobot_cv/hobotcv_imgproc.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim {

Yolo11Node::Yolo11Node(const std::string &node_name,
                       const rclcpp::NodeOptions &options)
    : hobot::dnn_node::DnnNode(node_name, options) {
  using std::placeholders::_1;

  this->declare_parameter<std::string>("image_topic", "/image_raw");
  std::string image_topic;
  this->get_parameter("image_topic", image_topic);

  if (Init() != 0) {
    throw std::runtime_error("Yolo11Node init failed");
  }

  if (GetModelInputSize(0, model_input_width_, model_input_height_) != 0) {
    RCLCPP_WARN(this->get_logger(),
                "GetModelInputSize failed, using defaults %dx%d",
                model_input_width_, model_input_height_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Model input size %dx%d",
                model_input_width_, model_input_height_);
  }

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&Yolo11Node::FeedImg, this, _1));
  RCLCPP_INFO(this->get_logger(), "Subscribed image topic: %s",
              image_topic.c_str());
}

int Yolo11Node::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output) {
  if (!rclcpp::ok() || !node_output) {
    RCLCPP_WARN(this->get_logger(), "Invalid node_output in PostProcess");
    return -1;
  }

  rm_auto_aim::armors_keypoints.clear();

  std::shared_ptr<hobot::dnn_node::output_parser::DnnParserResult> det_result =
      nullptr;
  if (hobot::dnn_node::parser_yolov8::Parse(node_output, det_result) < 0 ||
      !det_result) {
    RCLCPP_ERROR(this->get_logger(), "Parse YOLOv8 output failed");
    return -1;
  }

  auto parser_output = std::dynamic_pointer_cast<DnnOutput>(node_output);
  float ratio = 1.0f;
  int original_w = model_input_width_;
  int original_h = model_input_height_;
  if (parser_output) {
    if (parser_output->ratio > 0.0f) {
      ratio = parser_output->ratio;
    }
    if (parser_output->img_w > 0) {
      original_w = parser_output->img_w;
    }
    if (parser_output->img_h > 0) {
      original_h = parser_output->img_h;
    }
  }

  const auto scale_x = [&](float value) {
    float scaled = value * ratio;
    if (original_w > 0) {
      scaled = std::clamp(scaled, 0.0f,
                          static_cast<float>(original_w - 1));
    }
    return scaled;
  };
  const auto scale_y = [&](float value) {
    float scaled = value * ratio;
    if (original_h > 0) {
      scaled = std::clamp(scaled, 0.0f,
                          static_cast<float>(original_h - 1));
    }
    return scaled;
  };

  for (const auto &det : det_result->perception.det) {
    const float xmin = scale_x(det.bbox.xmin);
    const float ymin = scale_y(det.bbox.ymin);
    const float xmax = scale_x(det.bbox.xmax);
    const float ymax = scale_y(det.bbox.ymax);

    std::vector<cv::Point2f> kps{
        {xmin, ymin},
        {xmax, ymin},
        {xmax, ymax},
        {xmin, ymax},
    };

    rm_auto_aim::ArmorDetection det_out;
    det_out.kpts = std::move(kps);
    det_out.class_name = det.class_name ? det.class_name : "";
    det_out.score = det.score;
    if (parser_output && parser_output->msg_header) {
      det_out.frame_id = parser_output->msg_header->frame_id;
      det_out.stamp_sec = parser_output->msg_header->stamp.sec;
      det_out.stamp_nanosec = parser_output->msg_header->stamp.nanosec;
    }
    rm_auto_aim::armors_keypoints.emplace_back(std::move(det_out));
  }

  RCLCPP_DEBUG(this->get_logger(),
               "PostProcess produced %zu detections",
               rm_auto_aim::armors_keypoints.size());
  return 0;
}

// 使用 hobotcv 对 NV12 图片做等比例 resize（保留宽高比），并返回 resized NV12 图片（存于 out_img）和 ratio
static int ResizeNV12Img(const char *in_img_data,
                  const int &in_img_height,
                  const int &in_img_width,
                  int &resized_img_height,
                  int &resized_img_width,
                  const int &scaled_img_height,
                  const int &scaled_img_width,
                  cv::Mat &out_img,
                  float &ratio) {
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void *)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w) {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  } else if (dst_ratio == ratio_h) {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }
  // hobot_cv要求输出宽度为16的倍数
  int remain = resized_width % 16;
  if (remain != 0) {
    //向下取16倍数，重新计算缩放系数
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  //高度向下取偶数
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  resized_img_height = resized_height;
  resized_img_width = resized_width;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

int Yolo11Node::SetNodePara() {
  RCLCPP_INFO(this->get_logger(), "Yolo11Node::SetNodePara()");
  // ensure dnn_node_para_ptr_ exists
  if (!dnn_node_para_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "dnn_node_para_ptr_ is null");
    return -1;
  }

  // Declare and read config_file parameter
  this->declare_parameter<std::string>("config_file", "");
  std::string config_file;
  this->get_parameter("config_file", config_file);
  // If user didn't pass config_file, try the repo default path
  if (config_file.empty()) {
    config_file = "autoaim/model/yolov11workconfig.json";
    RCLCPP_INFO(this->get_logger(), "No config_file param provided, trying default: %s", config_file.c_str());
  }

  std::string abs_config_path;
  try {
    abs_config_path = std::filesystem::absolute(config_file).string();
  } catch (...) {
    abs_config_path = config_file;
  }
  RCLCPP_INFO(this->get_logger(), "Trying parser config: %s", abs_config_path.c_str());

  if (!std::filesystem::exists(abs_config_path)) {
    RCLCPP_WARN(this->get_logger(), "Config file does not exist: %s -- parser will not be configured. Pass --ros-args -p config_file:=<path> to override.", abs_config_path.c_str());
    // Do not fail startup; allow node to run without parser configuration
    return 0;
  }

  std::ifstream ifs(abs_config_path.c_str());
  if (!ifs) {
    RCLCPP_ERROR(this->get_logger(), "Open config file [%s] failed", abs_config_path.c_str());
    return -1;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(this->get_logger(), "Parse config file [%s] failed", config_file.c_str());
    return -1;
  }

  int ret = hobot::dnn_node::parser_yolov8::LoadConfig(document);
  if (ret < 0) {
    RCLCPP_ERROR(this->get_logger(), "parser_yolov8::LoadConfig failed for %s", config_file.c_str());
    return -1;
  }

  // Populate basic dnn_node parameters if present
  if (document.HasMember("model_file")) {
    dnn_node_para_ptr_->model_file = document["model_file"].GetString();
  }
  if (document.HasMember("model_name")) {
    dnn_node_para_ptr_->model_name = document["model_name"].GetString();
  }
  if (document.HasMember("task_num")) {
    dnn_node_para_ptr_->task_num = document["task_num"].GetInt();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded parser config from %s", config_file.c_str());
  return 0;
}

void Yolo11Node::FeedImg(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!rclcpp::ok() || !img_msg) {
    
    return;
  }

  auto dnn_output = std::make_shared<DnnOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);

  // Fill metadata
  dnn_output->img_w = img_msg->width;
  dnn_output->img_h = img_msg->height;
  dnn_output->model_w = model_input_width_;
  dnn_output->model_h = model_input_height_;

  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  // 如果输入尺寸和模型输入不一致，先用 hobotcv 做等比例 resize（保留宽高比）
  if (static_cast<int>(img_msg->height) != model_input_height_ ||
      static_cast<int>(img_msg->width) != model_input_width_) {
    cv::Mat out_img;
    float ratio = 1.0f;
    int out_h = 0, out_w = 0;
    int ret = ResizeNV12Img(reinterpret_cast<const char *>(img_msg->data.data()),
                            img_msg->height,
                            img_msg->width,
                            out_h,
                            out_w,
                            model_input_height_,
                            model_input_width_,
                            out_img,
                            ratio);
    if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("yolo11_node"), "Resize nv12 img fail");
      return;
    }

    // hobotcv 返回的 out_img 为 NV12 存储在 Mat 中，height = H', width = W'
    // 对于 NV12，实际像素高度为 rows * 2 / 3（因为 Mat 存储 Y + UV）
    int out_img_width = out_img.cols;
    int out_img_height = out_img.rows * 2 / 3;

    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(out_img.data),
        out_img_height,
        out_img_width,
        model_input_height_,
        model_input_width_);

    dnn_output->ratio = ratio;
    dnn_output->resized_w = out_img_width;
    dnn_output->resized_h = out_img_height;
  } else {
    // 直接构造 pyramid
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
    dnn_output->ratio = 1.0f;
    dnn_output->resized_w = img_msg->width;
    dnn_output->resized_h = img_msg->height;
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo11_node"), "Get pyramid fail");
    return;
  }

  if (dnn_output->ratio != 1.0f) {
    // optionally cache pyramid for rendering
    dnn_output->pyramid = pyramid;
  }

  auto inputs = std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  if (Run(inputs, dnn_output, nullptr, false) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo11_node"), "Run inference fail!");
  }
}

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::Yolo11Node)
