#include "armor_detector/yolo11_node.hpp"
#include "armor_detector/armors_shared.hpp"
#include "armor_detector/armor.hpp"
#include "dnn_node/include/ucp/easy_dnn/data_structure.h"
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

Yolo11Node::Yolo11Node(const rclcpp::NodeOptions &options)
    : Yolo11Node("yolo11_node", options) {}

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
  // 使用官方解析器读取输出
  std::shared_ptr<hobot::dnn_node::output_parser::DnnParserResult> det_result = nullptr;
  if (hobot::dnn_node::parser_yolov8::Parse(node_output, det_result) < 0 || !det_result) {
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

  // 计算 letterbox 偏移量
  float scale_factor = 1.0f / ratio;  // letterbox 的实际缩放因子
  float x_offset = (model_input_width_ - original_w * scale_factor) / 2.0f;
  float y_offset = (model_input_height_ - original_h * scale_factor) / 2.0f;

  const auto scale_x = [&](float value) {
    float scaled = (value - x_offset) * ratio;
    if (original_w > 0) {
      scaled = std::clamp(scaled, 0.0f, static_cast<float>(original_w - 1));
    }
    return scaled;
  };
  const auto scale_y = [&](float value) {
    float scaled = (value - y_offset) * ratio;
    if (original_h > 0) {
      scaled = std::clamp(scaled, 0.0f, static_cast<float>(original_h - 1));
    }
    return scaled;
  };

  // 读取颜色过滤参数：-1 不过滤；0 仅红(R)；1 仅蓝(B)
  int detect_color = -1;
  (void)this->get_parameter("detect_color", detect_color);

  // 遍历解析器的检测结果，将 bbox 四角映射为关键点（并调整为 PnP 期望顺序：BL, TL, TR, BR）
  for (const auto &det : det_result->perception.det) {
    const float xmin = scale_x(det.bbox.xmin);
    const float ymin = scale_y(det.bbox.ymin);
    const float xmax = scale_x(det.bbox.xmax);
    const float ymax = scale_y(det.bbox.ymax);

    // PnP顺序：BL(xmin,ymax), TL(xmin,ymin), TR(xmax,ymin), BR(xmax,ymax)
    std::vector<cv::Point2f> kps{
        {xmin, ymax},
        {xmin, ymin},
        {xmax, ymin},
        {xmax, ymax},
    };
    // 类别名称映射与颜色过滤：
    // - 若解析器类名以 'R' 或 'B' 开头（如 R1/B1），可按 detect_color 过滤
    // - 其它类（G,O,Bs,Bb）不受颜色过滤影响
    std::string cls = det.class_name ? det.class_name : "";
    if (detect_color == 0) { // 仅红
      if (!cls.empty() && (cls[0] == 'B' || cls[0] == 'b')) {
        continue;
      }
    } else if (detect_color == 1) { // 仅蓝
      if (!cls.empty() && (cls[0] == 'R' || cls[0] == 'r')) {
        continue;
      }
    }

    // 确认类别在九类集合中，若解析器给出其它命名可在此做映射
    // 允许的集合：G,1,2,3,4,5,O,Bs,Bb 以及可能带颜色前缀的数字：R1..R5 / B1..B5
    static const std::unordered_set<std::string> allowed = {
      "G","1","2","3","4","5","O","Bs","Bb",
      "R1","R2","R3","R4","R5","B1","B2","B3","B4","B5"
    };
    if (!cls.empty() && allowed.find(cls) == allowed.end()) {
      // 简单映射：若仅是数字且需要保留，直接通过；其它未知类别丢弃
      if (!(cls.size() == 1 && std::isdigit(static_cast<unsigned char>(cls[0])))) {
        RCLCPP_DEBUG(this->get_logger(), "Skip detection with unknown class: %s", cls.c_str());
        continue;
      }
    }

    rm_auto_aim::ArmorDetection det_out;
    det_out.kpts = std::move(kps);
    det_out.class_name = cls;
    det_out.score = det.score;
    if (parser_output && parser_output->msg_header) {
      det_out.frame_id = parser_output->msg_header->frame_id;
      det_out.stamp_sec = parser_output->msg_header->stamp.sec;
      det_out.stamp_nanosec = parser_output->msg_header->stamp.nanosec;
    }
    rm_auto_aim::armors_keypoints.emplace_back(std::move(det_out));
  }

  RCLCPP_INFO(this->get_logger(),
               "PostProcess produced %zu detections",
               rm_auto_aim::armors_keypoints.size());
  
  // 打印每个检测结果的详细信息
  for (size_t i = 0; i < rm_auto_aim::armors_keypoints.size(); ++i) {
    const auto& detection = rm_auto_aim::armors_keypoints[i];
    RCLCPP_INFO(this->get_logger(),
                "Detection %zu: class=%s, score=%.3f, keypoints=%zu",
                i, detection.class_name.c_str(), detection.score, detection.kpts.size());
  }
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

  int ret = hobot::dnn_node::parser_yolov8::LoadConfig(document);
  if (ret < 0) {
    RCLCPP_ERROR(this->get_logger(), "parser_yolov8::LoadConfig failed for %s", config_file.c_str());
    return -1;
  }
  }

  auto dnn_output = std::make_shared<DnnOutput>();

  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  // 根据图像编码格式选择处理方式（仅保留 NV12 路径）
  if (img_msg->encoding == "nv12") {
    // NV12 格式处理：如果输入尺寸和模型输入不一致，先用 hobotcv 做等比例 resize（保留宽高比）
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

    // 对于 NV12，实际像素高度为 rows * 2 / 3（因为 Mat 存储 Y + UV）
    int out_img_width = out_img.cols;
    int out_img_height = out_img.rows * 2 / 3;
        det_out.class_name = cls;
        det_out.score = det.score;
        reinterpret_cast<const char *>(out_img.data),
        out_img_height,
        out_img_width,
        model_input_height_,
        model_input_width_);
        rm_auto_aim::armors_keypoints.emplace_back(std::move(det_out));
      }
  }

  auto inputs = std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  //初始化输出
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);

  // Fill metadata
  dnn_output->img_w = img_msg->width;
  dnn_output->img_h = img_msg->height;
  dnn_output->model_w = model_input_width_;
  dnn_output->model_h = model_input_height_;

  // if (dnn_output->ratio != 1.0f) {
  //   // optionally cache pyramid for rendering
  //   dnn_output->pyramid = pyramid;
  // }

  

  if (Run(inputs, dnn_output, nullptr, false) != 0
      && Run(inputs, dnn_output, nullptr, false) != HB_DNN_TASK_NUM_EXCEED_LIMIT) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo11_node"), "Run inference fail!");
  }
}

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::Yolo11Node)
