#ifndef ARMOR_DETECTOR__YOLO_HPP_
#define ARMOR_DETECTOR__YOLO_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "dnn/hb_dnn.h"

#include "dnn_node/dnn_node.h"
#include "armor_detector/armors_shared.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/mat.hpp>

namespace rm_auto_aim {

// 继承 DnnNodeOutput
struct DnnOutput : public hobot::dnn_node::DnnNodeOutput {
  // 前处理时的缩放比例（用于将 model-space 坐标映射回原图）
  float ratio = 1.0f;
  // 原始输入图像分辨率
  int img_w = 0;
  int img_h = 0;

  // 原图，用于后处理下游调试可视化
  cv::Mat image_bgr;

  // 模型输入分辨率
  int model_w = 0;
  int model_h = 0;

  // 预处理后实际送入模型的分辨率（hobotcv resize 后的尺寸）
  int resized_w = 0;
  int resized_h = 0;
  // letterbox/resize 偏移
  float x_offset = 0.0f;
  float y_offset = 0.0f;
};

class YoloNode : public hobot::dnn_node::DnnNode {
 public:
  YoloNode(const std::string &node_name = "yolo_node",
           const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  
  // 为组件支持添加的构造函数
  explicit YoloNode(const rclcpp::NodeOptions &options);

 protected:
  int SetNodePara() override;

  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &
                  node_output) override;

 private:
  int model_input_width_ = -1;
  int model_input_height_ = -1;
  float score_threshold_ = 0.65f;
  float nms_threshold_ = 0.45f;
  int detect_color_ = -1;
  hbDNNTensorProperties input_properties_{};
  bool has_input_properties_ = false;

  // 图片订阅
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  void FeedImg(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  void ProcessImage(const cv::Mat &image, const std_msgs::msg::Header &header);
  void UpdateFps();

  bool use_image_file_ = false;
  std::string image_file_path_;
  bool enable_fps_logging_ = false;
  rclcpp::Time last_frame_time_;
  bool has_last_frame_time_ = false;

  // 缓存最近一次检测结果，便于离线文件模式可视化
  std::vector<ArmorDetection> last_detections_;
  std::mutex last_det_mutex_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__YOLO_HPP_
