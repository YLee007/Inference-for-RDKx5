#include "armor_detector/yolo11_node.hpp"
#include "armor_detector/armors_shared.hpp"
#include <opencv2/core.hpp>

#include <algorithm>
#include <numeric>
#include <cmath>
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
  // 颜色过滤：-1 不过滤，0 只保留红，1 只保留蓝（与OpenVINO实现一致）
  this->declare_parameter<int>("detect_color", -1);

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

  // 使用自定义输出解析：
  // 0..7: 四个关键点(TL, BL, BR, TR) 的 x,y
  // 8: objectness (sigmoid)
  // 9..12: 颜色分支 (红、蓝、灰、紫)
  // 13..21: 数字/类别 (G,1,2,3,4,5,O,Bs,Bb)

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
    // letterbox 坐标映射：先减去偏移，再缩放到原图
    float scaled = (value - x_offset) * ratio;
    if (original_w > 0) {
      scaled = std::clamp(scaled, 0.0f,
                          static_cast<float>(original_w - 1));
    }
    return scaled;
  };
  const auto scale_y = [&](float value) {
    // letterbox 坐标映射：先减去偏移，再缩放到原图
    float scaled = (value - y_offset) * ratio;
    if (original_h > 0) {
      scaled = std::clamp(scaled, 0.0f,
                          static_cast<float>(original_h - 1));
    }
    return scaled;
  };

  if (node_output->output_tensors.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No output tensors to parse");
    return -1;
  }

  auto tensor = node_output->output_tensors[0];
  tensor->CACHE_INVALIDATE();

  int nd = tensor->properties.validShape.numDimensions;
  const auto *dim = tensor->properties.validShape.dimensionSize;
  int rows = 0, cols = 0;
  if (nd == 3) { rows = static_cast<int>(dim[1]); cols = static_cast<int>(dim[2]); }
  else if (nd == 2) { rows = static_cast<int>(dim[0]); cols = static_cast<int>(dim[1]); }
  else if (nd == 4) { rows = static_cast<int>(dim[2]); cols = static_cast<int>(dim[3]); }
  else { RCLCPP_ERROR(this->get_logger(), "Unsupported tensor dims: %d", nd); return -1; }
  if (cols < 22) { RCLCPP_ERROR(this->get_logger(), "Expect >=22 columns, got %d", cols); return -1; }

  auto *data = tensor->GetTensorData<float>();
  auto sigmoid = [](float x){ return 1.0f / (1.0f + std::exp(-x)); };

  const float conf_thr = 0.65f;
  const float nms_thr = 0.45f;
  int detect_color = -1;
  (void)this->get_parameter("detect_color", detect_color);

  struct Item { cv::Rect2f box; float class_score; float disp_score; rm_auto_aim::ArmorDetection det; };
  std::vector<Item> items; items.reserve(rows);

  for (int i = 0; i < rows; ++i) {
    const float *r = data + i * cols;
    float obj = sigmoid(r[8]);
    if (obj < conf_thr) continue;

    // 颜色 argmax 9..12
    int color_idx = 0; float color_max = r[9];
    for (int k = 10; k <= 12; ++k) if (r[k] > color_max) { color_max = r[k]; color_idx = k - 9; }
    if (color_idx >= 2) continue; // 丢弃灰/紫
    // OpenVINO风格的颜色过滤：detect_color==0 只保留红(丢弃蓝)，detect_color==1 只保留蓝(丢弃红)
    if (detect_color == 0 && color_idx == 1) continue; // 0: red mode, drop blue
    if (detect_color == 1 && color_idx == 0) continue; // 1: blue mode, drop red

    // 数字/类别 argmax 13..21
    int cls_idx = 0; float cls_max = r[13];
    for (int k = 14; k <= 21; ++k) if (r[k] > cls_max) { cls_max = r[k]; cls_idx = k - 13; }

    // 关键点：模型坐标->原图坐标 (输入顺序 TL, BL, BR, TR)
    float tlx = scale_x(r[0]); float tly = scale_y(r[1]);
    float blx = scale_x(r[2]); float bly = scale_y(r[3]);
    float brx = scale_x(r[4]); float bry = scale_y(r[5]);
    float trx = scale_x(r[6]); float try_ = scale_y(r[7]);

    // PnP 期望顺序：BL, TL, TR, BR
    std::vector<cv::Point2f> kps{
      {blx, bly}, {tlx, tly}, {trx, try_}, {brx, bry}
    };

    // 外接框用于 NMS
    float minx = std::min(std::min(tlx, blx), std::min(brx, trx));
    float maxx = std::max(std::max(tlx, blx), std::max(brx, trx));
    float miny = std::min(std::min(tly, bly), std::min(bry, try_));
    float maxy = std::max(std::max(tly, bly), std::max(bry, try_));
    cv::Rect2f rect(minx, miny, std::max(0.0f, maxx - minx), std::max(0.0f, maxy - miny));

    static const char* num_labels[9] = {"G","1","2","3","4","5","O","Bs","Bb"};
    std::string label = num_labels[cls_idx];
    std::string class_name;
    if (label == "1" || label == "2" || label == "3" || label == "4" || label == "5") {
      class_name = (color_idx == 0 ? "R" : "B");
      class_name += label;
    } else {
      class_name = label;
    }

    float class_score = cls_max;        // 用于 NMS 排序与保留阈值（与OpenVINO一致）
    float final_score = obj * cls_max;  // 显示分数，可保留以供上层参考

    rm_auto_aim::ArmorDetection det_out;
    det_out.kpts = std::move(kps);
    det_out.class_name = class_name;
    det_out.score = final_score;
    if (parser_output && parser_output->msg_header) {
      det_out.frame_id = parser_output->msg_header->frame_id;
      det_out.stamp_sec = parser_output->msg_header->stamp.sec;
      det_out.stamp_nanosec = parser_output->msg_header->stamp.nanosec;
    }
    items.push_back({rect, class_score, final_score, std::move(det_out)});
  }

  // 简单 NMS
  auto iou = [](const cv::Rect2f &a, const cv::Rect2f &b){
    float inter = (a & b).area();
    float uni = a.area() + b.area() - inter;
    return uni > 0 ? inter / uni : 0.0f;
  };
  std::vector<int> idx(items.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&](int i, int j){return items[i].class_score > items[j].class_score;});
  std::vector<char> suppressed(items.size(), 0);
  for (size_t m = 0; m < idx.size(); ++m) {
    int i = idx[m];
    if (suppressed[i]) continue;
    if (items[i].class_score < conf_thr) continue; // 与OpenVINO相同，NMS阈值基于类别分数
    rm_auto_aim::armors_keypoints.emplace_back(std::move(items[i].det));
    for (size_t n = m + 1; n < idx.size(); ++n) {
      int j = idx[n];
      if (suppressed[j]) continue;
      if (iou(items[i].box, items[j].box) > nms_thr) suppressed[j] = 1;
    }
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
    RCLCPP_DEBUG(this->get_logger(), "Get img failed");
    return;
  }

  auto dnn_output = std::make_shared<DnnOutput>();

  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  // Letterbox resize 函数
  auto letterbox_resize = [](const cv::Mat& img, int target_w, int target_h, 
                            float& scale, int& x_offset, int& y_offset) -> cv::Mat {
    int img_w = img.cols;
    int img_h = img.rows;
    
    // 计算缩放比例（保持宽高比）
    scale = std::min(static_cast<float>(target_w) / img_w, 
                     static_cast<float>(target_h) / img_h);
    
    // 计算缩放后的尺寸
    int new_w = static_cast<int>(img_w * scale);
    int new_h = static_cast<int>(img_h * scale);
    
    // resize 图像
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));
    
    // 创建目标尺寸的画布并填充灰色
    cv::Mat letterbox_img(target_h, target_w, img.type(), cv::Scalar(114, 114, 114));
    
    // 计算居中位置
    x_offset = (target_w - new_w) / 2;
    y_offset = (target_h - new_h) / 2;
    
    // 将 resize 后的图像放到画布中心
    resized.copyTo(letterbox_img(cv::Rect(x_offset, y_offset, new_w, new_h)));
    
    return letterbox_img;
  };

  // 根据图像编码格式选择处理方式
  if (img_msg->encoding == "rgb8") {
    // RGB8 格式处理：使用 letterbox resize
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");

    // 应用 letterbox resize
    float scale;
    int x_offset, y_offset;
    cv::Mat letterbox_img = letterbox_resize(cv_img->image, model_input_width_, model_input_height_,
                                           scale, x_offset, y_offset);

    // 设置输出参数
    dnn_output->ratio = 1.0f / scale;  // 用于坐标映射的缩放比例
    dnn_output->resized_w = model_input_width_;
    dnn_output->resized_h = model_input_height_;
    
    // 记录 letterbox 偏移量（需要在 PostProcess 中使用）
    dnn_output->img_w = cv_img->image.cols;  // 原图宽度
    dnn_output->img_h = cv_img->image.rows;  // 原图高度

    RCLCPP_DEBUG(this->get_logger(), 
                 "Letterbox: input=%dx%d, model=%dx%d, scale=%.3f, offset=(%d,%d), ratio=%.3f",
                 cv_img->image.cols, cv_img->image.rows,
                 model_input_width_, model_input_height_,
                 scale, x_offset, y_offset, dnn_output->ratio);

    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromBGRImg(
        letterbox_img, model_input_height_, model_input_width_);
    
  } else if (img_msg->encoding == "nv12") {
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
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("yolo11_node"), 
                 "Unsupported image encoding: %s", img_msg->encoding.c_str());
    return;
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo11_node"), "Get pyramid fail");
    return;
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
