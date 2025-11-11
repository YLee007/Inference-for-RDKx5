#ifndef AUTO_AIM__YOLO11_HPP
#define AUTO_AIM__YOLO11_HPP

// 标准库头文件（确保所有依赖类型声明）
#include <list>
#include <string>
#include <vector>
#include <memory>
#include <functional>

// OpenCV 头文件
#include <opencv2/opencv.hpp>

// RDK 相关头文件（RDK 原生图像类型）
#include "hobot_cv/image.h"

// dnn_node 封装头文件
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/dnn_node_impl.h"

// 自定义头文件（基类和装甲板类型）
#include "armor.hpp"
#include "yolo.hpp"

namespace rm_auto_aim
{
class YOLO11 : public YOLOBase
{
public:
  // 异步推理回调函数类型（与 cpp 完全一致）
  using InferCallback = std::function<void(std::list<Armor>)>;

  // 构造函数：通过配置文件初始化（参数类型、顺序匹配 cpp）
  YOLO11(const std::string & config_path, bool debug);

  // 析构函数：override 匹配基类虚析构
  ~YOLO11() override;

  // 同步推理接口（参数、返回值完全匹配 cpp）
  std::list<Armor> infer(const uint8_t* nv12_img_data, int img_width, int img_height,
                         int alloctask_timeout_ms = 1000, int infer_timeout_ms = 2000);

  // 异步推理接口（参数、返回值完全匹配 cpp）
  int infer_async(const uint8_t* nv12_img_data, int img_width, int img_height,
                  InferCallback callback, int alloctask_timeout_ms = 1000, int infer_timeout_ms = 2000);

  // 基类纯虚接口实现（参数、返回值匹配 cpp）
  std::list<Armor> detect(const std::shared_ptr<hobot::cv::Image> & hobot_img, int frame_count) override;

private:
  // 预处理：NV12 转 dnn_node 输入
  bool preprocess(const uint8_t* nv12_img_data, int img_width, int img_height,
                  std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>& dnn_inputs);

  // 后处理回调：适配 dnn_node 格式
  void postprocess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& output,
                   InferCallback callback = nullptr);

  // 解析输出张量：转 Armor 列表
  std::list<Armor> parse_output(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& output);

  // 装甲板类型判断
  ArmorType get_type(const Armor & armor);

  // 绘制检测结果（输入 cv::Mat）
  void draw_detections(const cv::Mat& img, const std::list<Armor> & armors);

  // 关键点排序
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);

  // 归一化中心坐标
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::shared_ptr<hobot::dnn_node::DnnNodePara> dnn_node_para_;    // dnn_node 配置参数
  std::shared_ptr<hobot::dnn_node::DnnNodeImpl> dnn_node_impl_;    // dnn_node 实现实例

  bool debug_;                  // 调试模式开关
  int input_w_;                 // 模型输入宽度（自动从模型获取）
  int input_h_;                 // 模型输入高度（自动从模型获取）
  int class_num_;               // 检测类别数（从配置读取）
  float min_confidence_;        // 置信度阈值（从配置读取）
  float nms_threshold_;         // NMS 阈值（从配置读取）
  bool use_roi_ = false;        // 是否启用 ROI 推理（默认 false）
  std::string save_path_;       // 调试图像保存路径（从配置读取）
  int frame_count_ = 0;         // 帧计数（默认 0，用于调试保存）

  int src_img_w_;               // 原始图像宽度
  int src_img_h_;               // 原始图像高度
  float scale_ = 1.0f;          // 图像缩放比例（默认 1.0）
  cv::Point2f offset_;          // 图像偏移量（用于坐标还原）
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM__YOLO11_HPP