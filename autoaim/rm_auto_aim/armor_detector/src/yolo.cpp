#include "armor_detector/yolo.hpp"
#include <yaml-cpp/yaml.h>
#include "armor_detector/yolo11.hpp"  // 包含 YOLO11 头文件

namespace rm_auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  // 读取配置文件
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  // 根据配置选择 YOLO 类型
  if (yolo_name == "yolo11") {
    // 如果配置文件指定为 yolo11，则实例化 YOLO11
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }
  else {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  // 调用 YOLO11 对象的 detect 方法
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 调用 YOLO11 对象的 postprocess 方法
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace rm_auto_aim
