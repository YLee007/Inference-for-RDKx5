#ifndef ARMOR_DETECTOR__YOLO11_NODE_HPP_
#define ARMOR_DETECTOR__YOLO11_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "dnn_node/dnn_node.h"
#include "armor_detector/armors_shared.hpp"
#include <sensor_msgs/msg/image.hpp>

namespace rm_auto_aim {

// 继承 DnnNodeOutput
struct DnnOutput : public hobot::dnn_node::DnnNodeOutput {
  // 前处理时的缩放比例（用于将 model-space 坐标映射回原图）
  float ratio = 1.0f;
  // 原始输入图像分辨率
  int img_w = 0;
  int img_h = 0;

  // 模型输入分辨率
  int model_w = 0;
  int model_h = 0;

  // 预处理后实际送入模型的分辨率（hobotcv resize 后的尺寸）
  int resized_w = 0;
  int resized_h = 0;
  // 缓存用于本地渲染的 pyramid
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
};

class Yolo11Node : public hobot::dnn_node::DnnNode {
 public:
  Yolo11Node(const std::string &node_name = "yolo11_node",
             const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  
  // 为组件支持添加的构造函数
  explicit Yolo11Node(const rclcpp::NodeOptions &options);

 protected:
  int SetNodePara() override;

  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &
                  node_output) override;

 private:
  int model_input_width_ = -1;
  int model_input_height_ = -1;

  // 图片订阅
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  void FeedImg(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__YOLO11_NODE_HPP_
