#include "armor_detector/yolo11_node.hpp"
#include "armor_detector/armors_shared.hpp"
#include "armor_detector/armor.hpp"
#include "easy_dnn/data_structure.h"
#include <opencv2/core.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <unordered_set>
#include <cctype>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

#include <cmath>
#include "dnn_node/util/image_proc.h"
#include "hobot_cv/hobotcv_imgproc.h"
#include "rclcpp_components/register_node_macro.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

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

int Yolo11Node::SetNodePara() {
  // 声明并获取解析器配置文件参数；默认指向安装后的包内 model 目录
  this->declare_parameter<std::string>("parser_config", "model/yolov11workconfig.json");
  std::string config_file;
  this->get_parameter("parser_config", config_file);

  // 支持相对路径：优先按当前工作目录解析；若不存在，回退到包内 share 路径
  std::string abs_config_path = config_file;
  if (!std::filesystem::exists(abs_config_path)) {
    try {
      auto share_dir = ament_index_cpp::get_package_share_directory("armor_detector");
      auto try_path = (std::filesystem::path(share_dir) / config_file).string();
      if (std::filesystem::exists(try_path)) {
        abs_config_path = try_path;
      } else {
        // 兼容常见布局：config 放在 share 根或 model 子目录
        auto alt_path = (std::filesystem::path(share_dir) / "model" / std::filesystem::path(config_file).filename()).string();
        if (std::filesystem::exists(alt_path)) {
          abs_config_path = alt_path;
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to query package share dir: %s", e.what());
    }
  }

  // 读取并加载自定义解析配置（可选：class_names 列表）
  if (std::filesystem::exists(abs_config_path)) {
    std::ifstream ifs(abs_config_path.c_str());
    if (!ifs) {
      RCLCPP_ERROR(this->get_logger(), "Open config file [%s] failed", abs_config_path.c_str());
      return -1;
    }
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document document;
    document.ParseStream(isw);
    if (document.HasParseError()) {
      RCLCPP_ERROR(this->get_logger(), "Parse config file [%s] failed", abs_config_path.c_str());
      return -1;
    }
    // 模型加载必要参数
    if (document.HasMember("model_file") && document["model_file"].IsString()) {
      dnn_node_para_ptr_->model_file = document["model_file"].GetString();
    }
    if (document.HasMember("model_name") && document["model_name"].IsString()) {
      dnn_node_para_ptr_->model_name = document["model_name"].GetString();
    }
    if (document.HasMember("task_num") && document["task_num"].IsInt()) {
      dnn_node_para_ptr_->task_num = document["task_num"].GetInt();
    }
    // 常规模型类型：非ROI，直接推理
    dnn_node_para_ptr_->model_task_type = hobot::dnn_node::ModelTaskType::ModelInferType;

    if (document.HasMember("class_num") && document["class_num"].IsInt()) {
      custom_cfg_.class_num = document["class_num"].GetInt();
    }
    if (document.HasMember("score_threshold") && document["score_threshold"].IsNumber()) {
      custom_cfg_.score_threshold = static_cast<float>(document["score_threshold"].GetDouble());
    }
    if (document.HasMember("cls_names_list") && document["cls_names_list"].IsString()) {
      std::ifstream names_ifs(document["cls_names_list"].GetString());
      if (names_ifs) {
        std::string line;
        while (std::getline(names_ifs, line)) {
          if (!line.empty()) class_names_.push_back(line);
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded parser config: %s", abs_config_path.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Parser config not found: %s; using defaults", abs_config_path.c_str());
  }

  // 声明颜色过滤参数，默认不过滤
  if (!this->has_parameter("detect_color")) {
    this->declare_parameter<int>("detect_color", -1);
  }
  return 0;
}

int Yolo11Node::PostProcess(
  const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output) {
  if (!rclcpp::ok() || !node_output) {
    RCLCPP_WARN(this->get_logger(), "Invalid node_output in PostProcess");
    return -1;
  }

  rm_auto_aim::armors_keypoints.clear();
  // 使用自定义解析读取输出
  std::vector<rm_auto_aim::ArmorDetection> parsed;
  if (CustomParse(node_output, parsed) != 0) {
    RCLCPP_ERROR(this->get_logger(), "CustomParse failed");
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

  // 遍历自定义解析的检测结果，将四角映射为关键点（BL, TL, TR, BR）
  for (const auto &det : parsed) {
    const float xmin = scale_x(det.kpts[1].x); // TL.x
    const float ymin = scale_y(det.kpts[1].y); // TL.y
    const float xmax = scale_x(det.kpts[2].x); // TR.x
    const float ymax = scale_y(det.kpts[0].y); // BL.y

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
    std::string cls = det.class_name;
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

static inline float Sigmoid(float x) {
  return 1.0f / (1.0f + std::exp(-x));
}

int Yolo11Node::CustomParse(
  const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
  std::vector<rm_auto_aim::ArmorDetection> &detections) {
  detections.clear();
  if (!node_output) return -1;
  const auto &tensors = node_output->output_tensors;
  if (tensors.empty() || !tensors[0]) {
    RCLCPP_WARN(this->get_logger(), "No output tensors to parse");
    return 0;
  }
  auto tensor = tensors[0];
  // 直接访问底层内存，不依赖特定便捷方法
  auto *data = reinterpret_cast<float*>(tensor->sysMem[0].virAddr);
  hbDNNTensorProperties &prop = tensor->properties;

  // 期望形状类似 [1, rows, cols]，其中 cols >= 8+1+color_num+class_num
  int rows = prop.validShape.dimensionSize[1];
  int cols = prop.validShape.dimensionSize[2];
  if (!data || rows <= 0 || cols <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid tensor data/shape");
    return -1;
  }

  const int conf_col = 8;
  const int color_start = 9;
  const int color_end = color_start + custom_cfg_.color_num; // 不含右端
  const int cls_start = color_end;
  const int cls_end = cls_start + custom_cfg_.class_num; // 不含右端
  if (cols < cls_end) {
    RCLCPP_ERROR(this->get_logger(), "Cols %d < required %d", cols, cls_end);
    return -1;
  }

  // 读取颜色过滤参数：-1 不过滤；0 仅红；1 仅蓝（按你模型的约定）
  int detect_color = -1;
  (void)this->get_parameter("detect_color", detect_color);

  for (int r = 0; r < rows; ++r) {
    const float conf_raw = data[r * cols + conf_col];
    const float confidence = Sigmoid(conf_raw);
    if (confidence < custom_cfg_.score_threshold) continue;

    // 颜色 one-hot 最大值索引
    int best_color = 0; float best_color_score = -1e9f;
    for (int c = color_start; c < color_end; ++c) {
      float s = data[r * cols + c];
      if (s > best_color_score) { best_color_score = s; best_color = c - color_start; }
    }
    // 颜色过滤：根据约定进行保留/丢弃
    if (detect_color == 0 && best_color == 1) { // 仅红时丢弃蓝（若约定 0红1蓝）
      continue;
    } else if (detect_color == 1 && best_color == 0) { // 仅蓝时丢弃红
      continue;
    }

    // 类别 one-hot 最大值索引
    int best_cls = 0; float best_cls_score = -1e9f;
    for (int c = cls_start; c < cls_end; ++c) {
      float s = data[r * cols + c];
      if (s > best_cls_score) { best_cls_score = s; best_cls = c - cls_start; }
    }

    // 关键点：输入为左上逆时针，输出重排为 BL,TL,TR,BR
    float TLx = data[r * cols + 0];
    float TLy = data[r * cols + 1];
    float TRx = data[r * cols + 2];
    float TRy = data[r * cols + 3];
    float BRx = data[r * cols + 4];
    float BRy = data[r * cols + 5];
    float BLx = data[r * cols + 6];
    float BLy = data[r * cols + 7];

    // 类别名
    std::string cls_name;
    if (!class_names_.empty() && best_cls >= 0 && best_cls < static_cast<int>(class_names_.size())) {
      cls_name = class_names_[best_cls];
    } else {
      cls_name = std::to_string(best_cls);
    }

    rm_auto_aim::ArmorDetection det;
    det.kpts = {
      cv::Point2f(BLx, BLy),
      cv::Point2f(TLx, TLy),
      cv::Point2f(TRx, TRy),
      cv::Point2f(BRx, BRy)
    };
    det.class_name = std::move(cls_name);
    det.score = confidence * std::max(0.0f, best_cls_score); // 组合分数
    detections.emplace_back(std::move(det));
  }

  return 0;
}

void Yolo11Node::FeedImg(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  // 1. 空消息校验
  if (!img_msg) {
    RCLCPP_WARN(this->get_logger(), "Empty image message");
    return;
  }

  // 2. RGB图像编码校验
  if (img_msg->encoding != std::string("rgb8")) {
    RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s (expect rgb8)", img_msg->encoding.c_str());
    return;
  }

  // 3. 将ROS RGB图像转为OpenCV Mat（RGB格式）
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion error: %s", e.what());
    return;
  }
  cv::Mat rgb_img = cv_ptr->image; // 原始RGB图像（cv::Mat，通道顺序RGB）

  // 4. 核心：RGB图像尺寸转换（等比例缩放+填充到模型输入尺寸）
  int original_h = rgb_img.rows;
  int original_w = rgb_img.cols;
  int target_h = model_input_height_;
  int target_w = model_input_width_;

  // 校验尺寸有效性
  if (original_h <= 0 || original_w <= 0 || target_h <= 0 || target_w <= 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid image size: original(%dx%d), target(%dx%d)",
                original_w, original_h, target_w, target_h);
    return;
  }

  // 4.1 计算等比例缩放后的尺寸（保持宽高比，不拉伸）
  auto resized = hobot::dnn_node::GetResizedImgShape(original_h, original_w, target_h, target_w);
  int resized_h = resized.first;
  int resized_w = resized.second;

  // 4.2 执行RGB图像缩放（OpenCV原生方法，保证RGB通道不变）
  cv::Mat resized_rgb_img;
  cv::resize(rgb_img, resized_rgb_img, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_LINEAR);

  // 4.3 填充到模型输入尺寸（中心填充，补黑边，和原NV12金字塔的填充逻辑一致）
  cv::Mat input_rgb_img(target_h, target_w, CV_8UC3, cv::Scalar(0, 0, 0)); // 初始化黑底
  int x_offset = (target_w - resized_w) / 2;
  int y_offset = (target_h - resized_h) / 2;
  resized_rgb_img.copyTo(input_rgb_img(cv::Rect(x_offset, y_offset, resized_w, resized_h)));

  // 5. 将RGB Mat转为地平线DNNTensor（模型推理输入）
  float ratio = 1.0f;
  hbDNNTensorProperties tensor_properties; 
  auto dnn_tensor = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
      input_rgb_img, target_h, target_w, tensor_properties, ratio, hobot::dnn_node::ImageType::RGB, true, false, false);

  if (!dnn_tensor) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create DNNTensor from RGB image");
    return;
  }

  // 6. 执行模型推理
  std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>> inputs{dnn_tensor};
  auto dnn_output = std::make_shared<DnnOutput>(); // 用于接收推理输出

  int run_ret = Run(inputs, dnn_output, false);
  if (run_ret != 0 && run_ret != HB_DNN_TASK_NUM_EXCEED_LIMIT) {
    RCLCPP_ERROR(this->get_logger(), "Run inference fail! ret=%d", run_ret);
    return;
  }

  // 7. （可选）获取推理输出并验证（如需处理输出，可在此调用PostProcess）
  RCLCPP_INFO(this->get_logger(), "Inference success! Image resized from (%dx%d) to (%dx%d)",
              original_w, original_h, target_w, target_h);
  
  // 若需要处理推理输出，取消下面注释（调用原有PostProcess）
  // PostProcess(dnn_output);
}

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::Yolo11Node)
