#include "armor_detector/yolo11.hpp"
#include <fmt/chrono.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

#include "hb_dnn/hb_dnn.h"
#include "hb_dnn/hb_dnn_tensor.h"
#include "hb_dnn/hb_dnn_image.h"
#include "hb_dnn/hb_dnn_error.h"
#include "hobot_cv/image.h"
#include "hobot_cv/image_manager.h"


namespace rm_auto_aim
{
// 构造函数（初始化 hb_dnn）
YOLO11::YOLO11(const std::string & config_path, bool debug)
: debug_(debug), dnn_handle_(nullptr), input_h_(640), input_w_(640)
{
  // 读取配置文件
  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    tools::logger()->error("Failed to open config.json");
    throw std::runtime_error("Config file open failed");
  }

  nlohmann::json json_config;
  config_file >> json_config;

  // 提取配置参数
  model_path_ = json_config["model_file"];  
  int task_num = json_config["task_num"];
  std::string dnn_parser = json_config["dnn_Parser"];
  int model_output_count = json_config["model_output_count"];
  int reg_max = json_config["reg_max"];
  class_num_ = json_config["class_num"];  // 覆盖类内常量，从配置读取
  std::string cls_names_list = json_config["cls_names_list"];
  
  std::vector<int> strides = json_config["strides"].get<std::vector<int>>();
  std::vector<int> output_order = json_config["output_order"].get<std::vector<int>>();
  
  min_confidence_ = json_config["score_threshold"];  // 复用配置的阈值
  nms_threshold_ = json_config["nms_threshold"];
  int nms_top_k = json_config["nms_top_k"];

  // 初始化 RDK hb_dnn（NPU 推理） 
  hbDNNInitParam init_param = {0};
  init_param.model_path = model_path_.c_str();  
  init_param.device_id = 0;  // 默认 NPU 设备 ID

  // 1. 创建 hb_dnn 句柄
  int ret = hb_dnn_init(&dnn_handle_, &init_param);
  if (ret != 0) {
    tools::logger()->error("hb_dnn_init failed! ret={}, msg={}", ret, hb_dnn_get_error_msg(ret));
    throw std::runtime_error(fmt::format("hb_dnn init failed, ret={}", ret));
  }

  // 2. 获取模型信息（自动适配输入尺寸，覆盖默认值）
  hbDNNModelInfo model_info;
  ret = hb_dnn_get_model_info(&model_info, dnn_handle_);
  if (ret != 0) {
    tools::logger()->error("hb_dnn_get_model_info failed! ret={}", ret);
    hb_dnn_destroy(&dnn_handle_);
    throw std::runtime_error("Get model info failed");
  }

  // 模型输入格式：NCHW（batch, channel, height, width）
  input_w_ = model_info.input_tensor_info[0].shape.dim[3];
  input_h_ = model_info.input_tensor_info[0].shape.dim[2];
  tools::logger()->info("Model input size: {}x{}, class_num={}", input_w_, input_h_, class_num_);

  // 3. 初始化输入/输出张量
  ret = hb_dnn_tensor_init(&input_tensor_, &model_info.input_tensor_info[0]);
  if (ret != 0) {
    tools::logger()->error("input tensor init failed! ret={}", ret);
    hb_dnn_destroy(&dnn_handle_);
    throw std::runtime_error("Input tensor init failed");
  }

  ret = hb_dnn_tensor_init(&output_tensor_, &model_info.output_tensor_info[0]);
  if (ret != 0) {
    tools::logger()->error("output tensor init failed! ret={}", ret);
    hb_dnn_tensor_destroy(&input_tensor_);
    hb_dnn_destroy(&dnn_handle_);
    throw std::runtime_error("Output tensor init failed");
  }
}

// 析构函数（释放 hb_dnn 资源）
YOLO11::~YOLO11()
{
  if (dnn_handle_ != nullptr) {
    hb_dnn_tensor_destroy(&input_tensor_);
    hb_dnn_tensor_destroy(&output_tensor_);
    hb_dnn_destroy(&dnn_handle_);
    tools::logger()->info("hb_dnn resources released");
  }
}

// detect 函数（RDK 硬件加速推理）
std::list<Armor> YOLO11::detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count)
{
  // 检查输入有效性
  if (hobot_img == nullptr || hobot_img->data() == nullptr) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  // 保存原图用于绘制（临时转 cv::Mat）
  tmp_img_ = cv::Mat(
    hobot_img->height(),
    hobot_img->width(),
    CV_8UC3,
    hobot_img->data(),
    hobot_img->step()
  );

  //  ROI 裁剪（RDK 硬件加速）
  std::shared_ptr<hobotcv::Image> bgr_img;
  offset_ = cv::Point2f(0, 0);
  if (use_roi_) {
    int roi_w = (roi_.width == -1) ? hobot_img->width() : roi_.width;
    int roi_h = (roi_.height == -1) ? hobot_img->height() : roi_.height;
    
    // 检查 ROI 越界
    if (roi_.x < 0 || roi_.y < 0 || roi_.x + roi_w > hobot_img->width() || roi_.y + roi_h > hobot_img->height()) {
      tools::logger()->warn("ROI out of bounds, use original image");
      bgr_img = hobot_img;
    } else {
      hobotcv::Rect roi_rect(roi_.x, roi_.y, roi_w, roi_h);
      // RDK 硬件加速裁剪（零拷贝）
      bgr_img = hobotcv::ImageManager::Instance()->CropImage(hobot_img, roi_rect);
      offset_ = cv::Point2f(roi_.x, roi_.y);
    }
  } else {
    bgr_img = hobot_img;
  }

  auto x_scale = static_cast<double>(input_h_) / bgr_img->height();
  auto y_scale = static_cast<double>(input_w_) / bgr_img->width();
  auto scale = std::min(x_scale, y_scale);
  int resized_h = static_cast<int>(bgr_img->height() * scale);
  int resized_w = static_cast<int>(bgr_img->width() * scale);

// 创建 640x640 黑底画布（RDK 原生图像类型）
  std::shared_ptr<hobotcv::Image> input_img = hobotcv::ImageManager::Instance()->CreateImage(
    input_w_, input_h_, hobotcv::PixelFormat::BGR_888
  );
  if (input_img == nullptr) {
    tools::logger()->error("Create input image failed");
    return std::list<Armor>();
  }

// 缩放后图像在画布上的 ROI（居中放置）
  hobotcv::Rect resize_roi(
    (input_w_ - resized_w) / 2,
    (input_h_ - resized_h) / 2,
    resized_w,
    resized_h
  );

// RDK 硬件加速缩放（VE 模块）
  int ret = hobot_cv::hobotcv_resize(
    *bgr_img,
    *input_img->roi(resize_roi),
    cv::Size(resized_w, resized_h)
  );
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ResizeOperation"),
               "ResizeOperation failed, error code: %d", ret);
    return std::list<Armor>();
  }

// NPU 推理（核心加速环节）
  // 1. 零拷贝填充输入张量（直接映射 input_img 内存）
  ret = hb_dnn_tensor_fill_from_image(
    &input_tensor_,
    input_img.get(),
    HB_DNN_IMAGE_FORMAT_BGR,  // 与 input_img 格式一致
    HB_DNN_DATA_TYPE_UINT8    // 图像数据类型（8位无符号）
  );
  if (ret != 0) {
    tools::logger()->error("Fill input tensor failed! ret={}", ret);
    return std::list<Armor>();
  }

  // 2. 执行 NPU 推理
  hbDNNInferParam infer_param = {0};
  ret = hb_dnn_infer(dnn_handle_, &input_tensor_, &output_tensor_, &infer_param);
  if (ret != 0) {
    tools::logger()->error("hb_dnn_infer failed! ret={}, msg={}", ret, hb_dnn_get_error_msg(ret));
    return std::list<Armor>();
  }

// 后处理解析 
  return parse(scale, output_tensor_, hobot_img, frame_count);
}

// parse 函数（解析 NPU 输出张量） 
std::list<Armor> YOLO11::parse(
  double scale,
  const hbDNNTensor & output_tensor,
  const std::shared_ptr<hobotcv::Image> & hobot_img,
  int frame_count)
{
  // 1. 获取 NPU 输出数据指针和维度信息
  float * output_data = static_cast<float *>(hb_dnn_tensor_get_data(&output_tensor));
  if (output_data == nullptr) {
    tools::logger()->error("Output tensor data is null");
    return std::list<Armor>();
  }

  // 输出维度：[batch, num_boxes, 4+class_num+8]（YOLO11 输出格式）
  int batch_size = output_tensor.shape.dim[0];
  int num_boxes = output_tensor.shape.dim[1];
  int output_dim = output_tensor.shape.dim[2];

  std::vector<ArmorName> class_id;
  std::vector<float> confidences;
  std::vector<cv::Rect> bbox;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  // 2. 遍历所有预测框（复用原有解析逻辑）
  for (int r = 0; r < num_boxes; r++) {
    int box_offset = r * output_dim;
    float * box = output_data + box_offset;

    // 解析 xywh（边界框）、scores（类别置信度）、keypoints（关键点）
    auto xywh = box;                  // x, y, w, h
    auto scores = box + 4;            // 类别置信度（共 class_num_ 个）
    auto one_key_points = box + 4 + class_num_;  // 4个关键点（8个值）

    // 计算最大类别置信度
    float max_score = 0.0f;
    int max_cls_idx = 0;
    for (int cls = 0; cls < class_num_; cls++) {
      if (scores[cls] > max_score) {
        max_score = scores[cls];
        max_cls_idx = cls;
      }
    }

    // 置信度过滤
    if (max_score < min_confidence_) continue;

    // 解析边界框（还原到原图坐标）
    float x = xywh[0];
    float y = xywh[1];
    float w = xywh[2];
    float h = xywh[3];

    // 修正缩放和 ROI 偏移
    int left = static_cast<int>((x - 0.5 * w) / scale + offset_.x);
    int top = static_cast<int>((y - 0.5 * h) / scale + offset_.y);
    int width = static_cast<int>(w / scale);
    int height = static_cast<int>(h / scale);

    // 解析关键点（4个点）
    std::vector<cv::Point2f> armor_key_points;
    for (int i = 0; i < 4; i++) {
      float kp_x = (one_key_points[i * 2] / scale) + offset_.x;
      float kp_y = (one_key_points[i * 2 + 1] / scale) + offset_.y;
      armor_key_points.emplace_back(kp_x, kp_y);
    }

    // 保存结果
    class_id.push_back(static_cast<ArmorName>(max_cls_idx));
    confidences.push_back(max_score);
    bbox.emplace_back(left, top, width, height);
    armors_key_points.emplace_back(armor_key_points);
  }

  // 3. NMS 过滤（复用原有逻辑）
  std::vector<int> indices;
  cv::dnn::NMSBoxes(bbox, confidences, min_confidence_, nms_threshold_, indices);

  // 4. 构造 Armor 列表
  std::list<Armor> armors;
  for (const auto & i : indices) {
    sort_keypoints(armors_key_points[i]);
    if (use_roi_) {
      armors.emplace_back(class_id[i], confidences[i], bbox[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(class_id[i], confidences[i], bbox[i], armors_key_points[i]);
    }
    armors.back().type = get_type(armors.back());
  }

  // 调试模式绘制
  if (debug_) {
    draw_detections(hobot_img, armors, frame_count);
  }

  return armors;
}

// 原有函数适配（仅修改输入类型） 
ArmorType YOLO11::get_type(const Armor & armor)
{
  if (armor.class_id == ArmorName::B1 || armor.class_id == ArmorName::R1) {
    return ArmorType::LARGE;
  } else {
    return ArmorType::SMALL;
  }
}

void YOLO11::draw_detections(const std::shared_ptr<hobotcv::Image> & hobot_img, const std::list<Armor> & armors, int frame_count) const
{
  // 临时转 cv::Mat 绘制（无拷贝）
  cv::Mat tmp_img(
    hobot_img->height(),
    hobot_img->width(),
    CV_8UC3,
    hobot_img->data(),
    hobot_img->step()
  );

  for (const auto & armor : armors) {
    cv::rectangle(tmp_img, armor.bbox, cv::Scalar(255, 0, 0), 2);
    cv::putText(
      tmp_img,
      fmt::format("ID:{} {:.2f}", ARMOR_TYPE_STR[static_cast<int>(armor.type)], armor.confidence),
      armor.bbox.tl() - cv::Point(0, 5),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(255, 0, 0),
      2
    );
  }

  // 保存调试图像
  if (!save_path_.empty()) {
    std::filesystem::create_directories(save_path_);
    std::string filename = fmt::format("{}/frame_{:06d}.jpg", save_path_, frame_count);
    cv::imwrite(filename, tmp_img);
  }
}

void YOLO11::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{ 
  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });
}

cv::Point2f YOLO11::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return cv::Point2f(center.x / bgr_img.cols, center.y / bgr_img.rows);
}

}// namespace rm_auto_aim

