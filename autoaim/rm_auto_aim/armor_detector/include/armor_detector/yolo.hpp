#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>
#include <memory>  // 必须包含：用于 std::shared_ptr / std::unique_ptr
#include <list>    // 必须包含：用于返回值 std::list<Armor>

// 引入 RDK 图像类型头文件（hobotcv::Image 依赖）
#include "hobot_cv/image.h"

// 引入装甲板类头文件（Armor 类型依赖）
#include "armor.hpp"

namespace rm_auto_aim
{
// 纯虚基类：定义检测统一接口，支持多态扩展（后续可添加 YOLOv8/YOLOv12 等派生类）
class YOLOBase
{
public:
  // 纯虚检测接口：输入 RDK 原生图像 + 帧计数，返回装甲板列表
  // 所有派生类（如 YOLO11）必须实现此接口
  virtual std::list<Armor> detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count) = 0;

  // 虚析构函数：确保派生类（如 YOLO11）的析构函数能被正确调用，避免内存泄漏
  // 纯虚基类必须有虚析构，否则 delete 基类指针时会触发未定义行为
  virtual ~YOLOBase() = default;
};

// 封装类：对外提供统一调用入口，隐藏底层实现（符合 "接口隔离原则"）
class YOLO
{
public:
  // 构造函数：通过配置文件初始化，默认开启调试模式
  // 内部会实例化具体的派生类（如 YOLO11）
  YOLO(const std::string & config_path, bool debug = true);

  // 对外检测接口：参数与基类一致，转发给底层派生类实现
  // frame_count 默认值 -1，内部自动修正为 0（避免非法值）
  std::list<Armor> detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count = -1);

private:
  // 基类智能指针：指向具体的派生类实例（多态核心）
  // 编译时无需知道 YOLO11 细节，仅需声明（后续在 cpp 中包含 yolo11.hpp）
  std::unique_ptr<YOLOBase> yolo_;
};

}  

#endif  // AUTO_AIM__YOLO_HPP