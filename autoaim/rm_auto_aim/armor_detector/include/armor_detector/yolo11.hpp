#ifndef AUTO_AIM__YOLO11_HPP
#define AUTO_AIM__YOLO11_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>  // OpenCV DNN module
#include <string>
#include <vector>

#include "armor.hpp"
#include "detector.hpp"
#include "yolo.hpp"

namespace rm_auto_aim
{
class YOLO11 : public YOLOBase
{
public:
  // Constructor to initialize the YOLO model
  YOLO11(const std::string & config_path, bool debug);

  // Override the detect method for inference
  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  // Override postprocess method to filter and process detections
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  // Paths for model configuration and weights
  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_;

  const int class_num_ = 38;               // Number of classes in the model
  const float nms_threshold_ = 0.3;        // Non-maxima suppression threshold
  const float score_threshold_ = 0.7;      // Confidence threshold for detection
  double min_confidence_, binary_threshold_;

  cv::dnn::Net net_;  // OpenCV DNN network object

  cv::Rect roi_;      // Region of interest (ROI) for detection
  cv::Point2f offset_;
  cv::Mat tmp_img_;   // Temporary image storage

  Detector detector_;

  // Add frame count as a class member variable to track frames
  int frame_count_;   // Frame count (for tracking video frames)

  // Helper methods
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM__YOLO11_HPP

