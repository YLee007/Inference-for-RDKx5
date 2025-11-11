#include "yolo.hpp"
#include "yolo11.hpp"  // 引入具体派生类 YOLO11
#include "tools/logger.hpp"  // 日志工具（用于打印初始化/错误信息）
#include <stdexcept>  // 用于异常处理

namespace rm_auto_aim
{
// -------------------------- YOLO 构造函数实现 --------------------------
// 功能：初始化 YOLO 检测器，内部实例化 YOLO11 派生类
YOLO::YOLO(const std::string & config_path, bool debug)
{
  // 校验配置文件路径有效性
  if (config_path.empty()) {
    const std::string err_msg = "YOLO init failed: empty config path";
    tools::logger()->error(err_msg);
    throw std::invalid_argument(err_msg);
  }

  try {
    // 实例化具体的检测器（这里是 YOLO11，后续可替换为 YOLOv8/YOLOv12 等）
    // 直接将配置文件路径和调试模式传入 YOLO11 构造函数
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
    tools::logger()->info("YOLO detector initialized successfully!");
    tools::logger()->info("Config path: {}", config_path);
    tools::logger()->info("Debug mode: {}", debug ? "ON" : "OFF");
  } catch (const std::exception& e) {
    // 捕获 YOLO11 初始化过程中的异常（如配置文件打开失败、模型加载失败）
    const std::string err_msg = fmt::format("YOLO init failed: {}", e.what());
    tools::logger()->error(err_msg);
    throw std::runtime_error(err_msg);  // 向上抛出异常，让调用者处理
  }
}

// -------------------------- YOLO detect 接口实现 --------------------------
// 功能：转发检测请求到底层 YOLO11 实例，返回装甲板列表
std::list<Armor> YOLO::detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count)
{
  // 1. 检查检测器是否初始化成功（yolo_ 不为空）
  if (!yolo_) {
    tools::logger()->error("detect failed: YOLO detector not initialized");
    return {};
  }

  // 2. 检查输入图像是否有效（避免空指针）
  if (!hobot_img) {
    tools::logger()->error("detect failed: input hobotcv::Image is null");
    return {};
  }

  // 3. 修正帧计数（默认值 -1 改为 0，避免非法值导致调试保存异常）
  int valid_frame_count = frame_count;
  if (valid_frame_count < 0) {
    valid_frame_count = 0;
    tools::logger()->warn("Invalid frame_count (-1), corrected to 0");
  }

  // 4. 转发检测请求到 YOLO11 的 detect 接口（多态调用）
  try {
    tools::logger()->debug("Start detecting frame: {}", valid_frame_count);
    auto armors = yolo_->detect(hobot_img, valid_frame_count);
    tools::logger()->debug("Detect completed: {} armors found", armors.size());
    return armors;
  } catch (const std::exception& e) {
    // 捕获检测过程中的异常（如图像格式不支持、推理失败）
    tools::logger()->error("Detect failed for frame {}: {}", valid_frame_count, e.what());
    return {};  // 异常时返回空列表，避免上层崩溃
  }
}

}  // namespace rm_auto_aim