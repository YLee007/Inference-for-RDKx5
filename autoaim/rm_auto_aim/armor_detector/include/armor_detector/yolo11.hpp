#ifndef AUTO_AIM__YOLO11_HPP
#define AUTO_AIM__YOLO11_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>  // 用于 shared_ptr

#include "hb_dnn/hb_dnn.h"
#include "hb_dnn/hb_dnn_tensor.h"
#include "hb_dnn/hb_dnn_image.h"
#include "hobot_cv/image.h"       // RDK 原生图像类型
#include "hobot_cv/image_manager.h"// RDK 图像管理工具

#include "armor.hpp"
#include "yolo.hpp"

namespace rm_auto_aim
{
class YOLO11 : public YOLOBase
{
public:
  // Constructor to initialize the YOLO model
  YOLO11(const std::string & config_path, bool debug);

  // 修改 detect 输入为 RDK 原生图像类型
  std::list<Armor> detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count);

private:
  // Paths for model configuration and weights
  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_;

  const int class_num_ = 38;               // Number of classes in the model
  const float nms_threshold_ = 0.3;        // Non-maxima suppression threshold
  const float score_threshold_ = 0.7;      // Confidence threshold for detection
  double min_confidence_, binary_threshold_;

  //替换 OpenCV DNN 为 RDK hb_dnn 
  hbDNNHandle dnn_handle_ = nullptr;       // hb_dnn 核心句柄（管理模型生命周期）
  hbDNNTensor input_tensor_;               // 模型输入张量（NPU 输入）
  hbDNNTensor output_tensor_;              // 模型输出张量（NPU 输出）
  int input_h_ = 640;                      // 模型输入高度（从 model 自动获取）
  int input_w_ = 640;                      // 模型输入宽度（从 model 自动获取）

  cv::Rect roi_;      // Region of interest (ROI) for detection
  cv::Point2f offset_;
  cv::Mat tmp_img_;   // Temporary image storage（仅用于调试绘制）

  // Add frame count as a class member variable to track frames
  int frame_count_;   // Frame count (for tracking video frames)

  // Method to get normalized center coordinates
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  // 原输入：cv::Mat & output → 新输入：const hbDNNTensor & output_tensor
  std::list<Armor> parse(
    double scale, 
    const hbDNNTensor & output_tensor, 
    const std::shared_ptr<hobotcv::Image> & hobot_img, 
    int frame_count);
  
  ArmorType get_type(const Armor& armor);

  // Method to save armor detection images (for debugging)
  void save(const Armor & armor) const;

  //  修改 draw_detections 输入为 RDK 图像 
  void draw_detections(const std::shared_ptr<hobotcv::Image> & hobot_img, const std::list<Armor> & armors, int frame_count) const;

  // Method to sort the keypoints of armor (used for detecting bounding box corners)
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM__YOLO11_HPP

