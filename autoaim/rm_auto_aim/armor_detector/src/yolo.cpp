#include "armor_detector/yolo.hpp"
#include "armor_detector/armors_shared.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <numeric>
#include <cmath>
#include <cctype>
#include <functional>
#include <memory>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include "dnn_node/util/image_proc.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim {

YoloNode::YoloNode(const rclcpp::NodeOptions &options)
    : YoloNode("yolo_node", options) {}

YoloNode::YoloNode(const std::string &node_name,
                       const rclcpp::NodeOptions &options)
    : hobot::dnn_node::DnnNode(node_name, options) {
  using std::placeholders::_1;

  this->declare_parameter<std::string>("image_topic", "/image_raw");
  this->declare_parameter<std::string>("model_file", "/home/sunrise/Documents/Inference-for-RDKx5/autoaim/model/yolov5_bgr.bin");
  this->declare_parameter<std::string>("model_name", "");
  this->declare_parameter<int>("task_num", 2);
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<double>("score_threshold", 0.65);
  this->declare_parameter<double>("nms_threshold", 0.45);
  this->declare_parameter<int>("detect_color",1);
  this->declare_parameter<bool>("use_image_file", false);
  this->declare_parameter<std::string>("image_file_path", "");
  this->declare_parameter<bool>("enable_fps_logging", false);

  std::string image_topic;
  this->get_parameter("image_topic", image_topic);
  this->get_parameter("detect_color", detect_color_);
  this->get_parameter("use_image_file", use_image_file_);
  this->get_parameter("image_file_path", image_file_path_);
  this->get_parameter("enable_fps_logging", enable_fps_logging_);

  if (Init() != 0) {
    throw std::runtime_error("YoloNode init failed");
  }

  if (GetModelInputSize(0, model_input_width_, model_input_height_) != 0) {
    RCLCPP_WARN(this->get_logger(),
                "GetModelInputSize failed, using defaults %dx%d",
                model_input_width_, model_input_height_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Model input size %dx%d",
                model_input_width_, model_input_height_);
    auto *model = this->GetModel();
    if (model && model->GetInputTensorProperties(input_properties_, 0) == 0) {
      // // Force RGB NCHW int8 input: 1x3x640x640
      // model_input_width_ = 640;
      // model_input_height_ = 640;
      // input_properties_.tensorLayout = HB_DNN_LAYOUT_NCHW;
      // input_properties_.tensorType = HB_DNN_TENSOR_TYPE_S8;
      // input_properties_.validShape.numDimensions = 4;
      // input_properties_.validShape.dimensionSize[0] = 1;
      // input_properties_.validShape.dimensionSize[1] = 3;
      // input_properties_.validShape.dimensionSize[2] = model_input_height_;
      // input_properties_.validShape.dimensionSize[3] = model_input_width_;
      // input_properties_.alignedShape = input_properties_.validShape;
      has_input_properties_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Input layout=%d type=%d dims=[%d,%d,%d,%d]",
                  input_properties_.tensorLayout,
                  input_properties_.tensorType,
                  input_properties_.validShape.dimensionSize[0],
                  input_properties_.validShape.dimensionSize[1],
                  input_properties_.validShape.dimensionSize[2],
                  input_properties_.validShape.dimensionSize[3]);
    } else {
      RCLCPP_ERROR(this->get_logger(), "GetInputTensorProperties failed");
    }
  }

  if (use_image_file_) {
    namespace fs = std::filesystem;
    std::vector<std::string> image_files;
    try {
      fs::path path(image_file_path_);
      if (fs::is_directory(path)) {
        RCLCPP_INFO(this->get_logger(), "Using image folder for inference: %s",
                    image_file_path_.c_str());
        for (const auto &entry : fs::directory_iterator(path)) {
          if (!entry.is_regular_file()) {
            continue;
          }
          std::string ext = entry.path().extension().string();
          std::transform(ext.begin(), ext.end(), ext.begin(),
                         [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
          if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" ||
              ext == ".bmp" || ext == ".tif" || ext == ".tiff") {
            image_files.emplace_back(entry.path().string());
          }
        }
        std::sort(image_files.begin(), image_files.end());
      } else {
        image_files.emplace_back(image_file_path_);
        RCLCPP_INFO(this->get_logger(), "Using image file for inference: %s",
                    image_file_path_.c_str());
      }
    } catch (const fs::filesystem_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Path error: %s", e.what());
    }

    if (image_files.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No valid image files found in path: %s",
                   image_file_path_.c_str());
    } else {
      for (const auto &file : image_files) {
        cv::Mat image = cv::imread(file, cv::IMREAD_COLOR);
        if (image.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to read image file: %s",
                       file.c_str());
          continue;
        }
        std_msgs::msg::Header header;
        header.frame_id = fs::path(file).filename().string();
        header.stamp = this->now();
        ProcessImage(image, header);
      }
    }
  } else {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        std::bind(&YoloNode::FeedImg, this, _1));
    RCLCPP_INFO(this->get_logger(), "Subscribed image topic: %s",
                image_topic.c_str());
  }
}

int YoloNode::PostProcess(
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
  float x_offset = 0.0f;
  float y_offset = 0.0f;
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
    x_offset = parser_output->x_offset;
    y_offset = parser_output->y_offset;
  }

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
  if (nd == 3) {
    rows = static_cast<int>(dim[1]);
    cols = static_cast<int>(dim[2]);
  } else if (nd == 2) {
    rows = static_cast<int>(dim[0]);
    cols = static_cast<int>(dim[1]);
  } else if (nd == 4) {
    // 兼容 [1, 25200, 22, 1]（hobot 默认）和 [1, 1, 25200, 22] 等常见排列
    if (dim[0] == 1 && dim[3] == 1) {
      rows = static_cast<int>(dim[1]);
      cols = static_cast<int>(dim[2]);
    } else if (dim[0] == 1 && dim[1] == 1) {
      rows = static_cast<int>(dim[2]);
      cols = static_cast<int>(dim[3]);
    } else {
      rows = static_cast<int>(dim[2]);
      cols = static_cast<int>(dim[3]);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported tensor dims: %d", nd);
    return -1;
  }
  if (cols < 22) { RCLCPP_ERROR(this->get_logger(), "Expect >=22 columns, got %d", cols); return -1; }

  auto *data = tensor->GetTensorData<float>();
  auto sigmoid = [](float x){ return 1.0f / (1.0f + std::exp(-x)); };

  const float conf_thr = score_threshold_;
  const float nms_thr = nms_threshold_;
  const int detect_color = detect_color_;

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

    float final_score = obj;             // 与 OpenVINO 实现保持一致：置信度仅用 objectness
    float class_score = cls_max;         // NMS 依据类别分支得分（未 sigmoid）

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

int YoloNode::SetNodePara() {
  RCLCPP_INFO(this->get_logger(), "YoloNode::SetNodePara()");
  if (!dnn_node_para_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "dnn_node_para_ptr_ is null");
    return -1;
  }

  this->get_parameter("model_file", dnn_node_para_ptr_->model_file);
  this->get_parameter("model_name", dnn_node_para_ptr_->model_name);
  this->get_parameter("task_num", dnn_node_para_ptr_->task_num);
  this->get_parameter("score_threshold", score_threshold_);
  this->get_parameter("nms_threshold", nms_threshold_);
  this->get_parameter("detect_color", detect_color_);
  std::string legacy_config;
  this->get_parameter("config_file", legacy_config);
  if (!legacy_config.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "config_file parameter is provided but ignored; use ROS params instead");
  }

  RCLCPP_INFO(this->get_logger(),
              "Params: model_file=%s, model_name=%s, task_num=%d, "
              "score_threshold=%.3f, nms_threshold=%.3f, detect_color=%d",
              dnn_node_para_ptr_->model_file.c_str(),
              dnn_node_para_ptr_->model_name.c_str(),
              dnn_node_para_ptr_->task_num,
              score_threshold_,
              nms_threshold_,
              detect_color_);

  return 0;
}

void YoloNode::FeedImg(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!rclcpp::ok() || !img_msg) {
    RCLCPP_DEBUG(this->get_logger(), "Get img failed");
    return;
  }

  if (use_image_file_) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Image file mode enabled, skip subscribed frame");
    return;
  }

  if (img_msg->encoding != "bgr8") {
    RCLCPP_ERROR(rclcpp::get_logger("yolo_node"), 
                 "Unsupported image encoding: %s", img_msg->encoding.c_str());
    return;
  }

  auto cv_img =
      cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");

  ProcessImage(cv_img->image, img_msg->header);
}

void YoloNode::ProcessImage(const cv::Mat &image,
                            const std_msgs::msg::Header &header) {
  if (!has_input_properties_) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo_node"),
                 "Input tensor properties not ready");
    return;
  }

  auto dnn_output = std::make_shared<DnnOutput>();

  std::shared_ptr<hobot::dnn_node::DNNTensor> input_tensor = nullptr;

  auto letterbox_resize = [](const cv::Mat &img, int target_w, int target_h,
                             float &scale, int &x_offset,
                             int &y_offset) -> cv::Mat {
    int img_w = img.cols;
    int img_h = img.rows;

    scale = std::min(static_cast<float>(target_w) / img_w,
                     static_cast<float>(target_h) / img_h);

    int new_w = static_cast<int>(img_w * scale);
    int new_h = static_cast<int>(img_h * scale);

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));

    cv::Mat letterbox_img(target_h, target_w, img.type(),
                          cv::Scalar(114, 114, 114));

    x_offset = (target_w - new_w) / 2;
    y_offset = (target_h - new_h) / 2;

    resized.copyTo(letterbox_img(cv::Rect(x_offset, y_offset, new_w, new_h)));

    return letterbox_img;
  };

  float scale;
  int x_offset, y_offset;
  cv::Mat letterbox_img =
      letterbox_resize(image, model_input_width_, model_input_height_, scale,
                       x_offset, y_offset);

  dnn_output->ratio = 1.0f / scale;
  dnn_output->resized_w = model_input_width_;
  dnn_output->resized_h = model_input_height_;
  dnn_output->img_w = image.cols;
  dnn_output->img_h = image.rows;
  dnn_output->x_offset = static_cast<float>(x_offset);
  dnn_output->y_offset = static_cast<float>(y_offset);

  RCLCPP_DEBUG(this->get_logger(),
               "Letterbox: input=%dx%d, model=%dx%d, scale=%.3f, offset=(%d,%d), ratio=%.3f",
               image.cols, image.rows, model_input_width_, model_input_height_,
               scale, x_offset, y_offset, dnn_output->ratio);

  float tensor_ratio = 1.0f;
  input_tensor = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
      letterbox_img, model_input_height_, model_input_width_, input_properties_,
      tensor_ratio, hobot::dnn_node::ImageType::RGB, false, false, false);

  if (!input_tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo_node"), "Get input tensor fail");
    return;
  }

  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>>{input_tensor};

  dnn_output->msg_header =
      std::make_shared<std_msgs::msg::Header>(header);

  dnn_output->model_w = model_input_width_;
  dnn_output->model_h = model_input_height_;

  const bool sync_mode = use_image_file_;
  int ret = Run(inputs, dnn_output, sync_mode);
  if (ret != 0 && ret != HB_DNN_TASK_NUM_EXCEED_LIMIT) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo_node"), "Run inference fail!");
    return;
  }

  UpdateFps();
}

void YoloNode::UpdateFps() {
  if (!enable_fps_logging_) {
    return;
  }
  auto now = this->now();
  if (has_last_frame_time_) {
    double dt = (now - last_frame_time_).seconds();
    if (dt > 0.0) {
      double fps = 1.0 / dt;
      RCLCPP_INFO(this->get_logger(), "Current FPS: %.2f", fps);
    }
  }
  last_frame_time_ = now;
  has_last_frame_time_ = true;
}

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::YoloNode)
