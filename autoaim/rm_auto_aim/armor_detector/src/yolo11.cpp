#include "armor_detector/yolo11.hpp"
#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <opencv2/dnn.hpp>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace rm_auto_aim
{
YOLO11::YOLO11(const std::string & config_path, bool debug)
: debug_(debug)
{
  // 读取配置文件
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolo11_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();

  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  // 使用 OpenCV DNN 读取模型
  net_ = cv::dnn::readNet(model_path_);
}

std::list<Armor> YOLO11::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  tmp_img_ = raw_img;
  if (use_roi_) {
    if (roi_.width == -1) {  // -1 表示该维度不裁切
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {  // -1 表示该维度不裁切
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // 预处理
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  
  // 转换为 DNN 输入格式
  cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);

  // 推理
  net_.setInput(blob);
  cv::Mat output = net_.forward();

  // 后处理
  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLO11::parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  cv::transpose(output, output);

  std::vector<ArmorName> class_id;
  std::vector<float> confidences;
  std::vector<cv::Rect> bbox;  // 改为 bbox，保存装甲板边界框
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    auto xywh = output.row(r).colRange(0, 4);
    auto scores = output.row(r).colRange(4, 4 + class_num_);
    auto one_key_points = output.row(r).colRange(4 + class_num_, 50);

    std::vector<cv::Point2f> armor_key_points;

    double score;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

    if (score < min_confidence_) continue;  // 如果置信度低于阈值则跳过

    // 获取边界框参数
    auto x = xywh.at<float>(0);
    auto y = xywh.at<float>(1);
    auto w = xywh.at<float>(2);
    auto h = xywh.at<float>(3);
    auto left = static_cast<int>((x - 0.5 * w) / scale);
    auto top = static_cast<int>((y - 0.5 * h) / scale);
    auto width = static_cast<int>(w / scale);
    auto height = static_cast<int>(h / scale);

    // 获取装甲板的关键点
    for (int i = 0; i < 4; i++) {
      float x = one_key_points.at<float>(0, i * 2 + 0) / scale;
      float y = one_key_points.at<float>(0, i * 2 + 1) / scale;
      cv::Point2f kp = {x, y};
      armor_key_points.push_back(kp);
    }
    armors_key_points.emplace_back(armor_key_points);
    confidences.emplace_back(score);  // 使用检测得到的置信度
    bbox.emplace_back(left, top, width, height);  // 保存装甲板的边界框

    // 确保 class_id 获取正确
    ArmorName detected_class = static_cast<ArmorName>(max_point.x);  // 通过 max_point 获取类别ID
    class_id.push_back(detected_class);
  }

  // 进行非最大抑制 (NMS) 来过滤重复框
  std::vector<int> indices;
  cv::dnn::NMSBoxes(bbox, confidences, min_confidence_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    sort_keypoints(armors_key_points[i]);
    if (use_roi_) {
      armors.emplace_back(class_id[i], confidences[i], bbox[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(class_id[i], confidences[i], bbox[i], armors_key_points[i]);
    }
  }

  // 调试模式下显示检测结果
  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

ArmorType YOLO11::get_type(const Armor & armor)
{
  if (armor.class_id == ArmorName::B1 || armor.class_id == ArmorName::R1) {
    return ArmorType::LARGE;
  } else {
    return ArmorType::SMALL;
  }
}

void YOLO11::draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  cv::Mat tmp_img = img.clone();

  for (const auto & armor : armors) {
    cv::rectangle(tmp_img, armor.bbox, cv::Scalar(255, 0, 0), 2);
    cv::putText(tmp_img, fmt::format("ID:{} {:.2f}", ARMOR_TYPE_STR[static_cast<int>(armor.type)], armor.confidence),
                armor.bbox.tl() - cv::Point(0, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
  }

  std::string filename = fmt::format("{}/frame_{}.jpg", save_path_, frame_count);
  cv::imwrite(filename, tmp_img);
}

void YOLO11::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;  // 按 x 坐标排序
  });
}

cv::Point2f YOLO11::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return cv::Point2f(center.x / bgr_img.cols, center.y / bgr_img.rows);
}

}  // namespace rm_auto_aim
