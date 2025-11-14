#include "armor_detector/yolo11.hpp"
#include "yolo.hpp"
#include <fmt/chrono.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

// STD
#include <cstdint> // For uint8_t
#include <functional>
#include <vector>

// 引入 dnn_node 相关头文件
#include "dnn_node/dnn_node_impl.h"

namespace rm_auto_aim
{
// 装甲板类型字符串映射（需确保 ArmorType 枚举与字符串对应）
const std::vector<std::string> ARMOR_TYPE_STR = {"SMALL", "LARGE"};

class YOLO11 : public YOLOBase
{
public:
  // 构造函数：通过配置文件初始化 dnn_node 和模型参数
  YOLO11(const std::string & config_path, bool debug);
  ~YOLO11();

  // 同步推理接口：输入 NV12 图像，阻塞返回检测结果
  std::vector<Armor> infer(const std::uint8_t* nv12_img_data, int img_width, int img_height,
                         int alloctask_timeout_ms = 1000, int infer_timeout_ms = 2000);

  // 异步推理接口：输入 NV12 图像，通过回调返回结果（非阻塞）
  using InferCallback = std::function<void(std::vector<Armor>)>;
  int infer_async(const std::uint8_t* nv12_img_data, int img_width, int img_height,
                  InferCallback callback, int alloctask_timeout_ms = 1000, int infer_timeout_ms = 2000);

  // 实现基类纯虚接口：输入 RDK 原生图像，返回装甲板列表
  std::vector<Armor> detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count) override;

private:
  // 预处理：NV12 图像转 dnn_node 所需的 DNNInput（NV12PyramidInput）
  bool preprocess(const std::uint8_t* nv12_img_data, int img_width, int img_height,
                  std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>& dnn_inputs);

  // 后处理回调：适配 dnn_node_impl 的 PostProcessCbType 格式
  void postprocess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& output,
                   InferCallback callback = nullptr);

  // 解析输出张量：将 dnn_node 输出转为 Armor 列表
  std::vector<Armor> parse_output(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& output);

  // 原有辅助函数（适配修改）
  ArmorType get_type(const Armor & armor);
  void draw_detections(const cv::Mat& img, const std::vector<Armor> & armors);
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  // -------------------------- dnn_node 核心成员 --------------------------
  std::shared_ptr<hobot::dnn_node::DnnNodePara> dnn_node_para_;    // dnn_node 配置参数
  std::shared_ptr<hobot::dnn_node::DnnNodeImpl> dnn_node_impl_;    // dnn_node 实现实例

  // -------------------------- 模型/检测配置参数 --------------------------
  bool debug_;                  // 调试模式开关
  int input_w_;                 // 模型输入宽度（自动从模型获取）
  int input_h_;                 // 模型输入高度（自动从模型获取）
  int class_num_;               // 检测类别数（从配置读取）
  float min_confidence_;        // 置信度阈值（从配置读取）
  float nms_threshold_;         // NMS 阈值（从配置读取）
  bool use_roi_ = false;        // 是否启用 ROI 推理（从配置读取）
  std::string save_path_;       // 调试图像保存路径（从配置读取）
  int frame_count_ = 0;         // 帧计数（用于调试保存）

  // -------------------------- 预处理相关参数 --------------------------
  int src_img_w_;               // 原始图像宽度
  int src_img_h_;               // 原始图像高度
  float scale_ = 1.0f;          // 图像缩放比例（原图 -> 模型输入）
  cv::Point2f offset_;          // 图像偏移量（用于还原边界框到原图）
};

// 构造函数实现：初始化 dnn_node 和模型
YOLO11::YOLO11(const std::string & config_path, bool debug)
: debug_(debug), input_w_(640), input_h_(640)
{
  // 1. 读取配置文件
  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    tools::logger()->error("Failed to open config.json: {}", config_path);
    throw std::runtime_error("Config file open failed");
  }
  nlohmann::json json_config;
  config_file >> json_config;

  // 2. 解析配置参数
  std::string model_file = json_config["model_file"];
  std::string model_name = json_config.value("model_name", "");  // 可选：多模型时指定
  int task_num = json_config["task_num"];                        // 并发任务数
  class_num_ = json_config["class_num"];                         // 检测类别数
  min_confidence_ = json_config["score_threshold"];              // 置信度阈值
  nms_threshold_ = json_config["nms_threshold"];                 // NMS 阈值
  use_roi_ = json_config.value("use_roi", false);                // 可选：是否启用 ROI 推理
  save_path_ = json_config.value("save_path", "");               // 可选：调试图像保存路径
  std::vector<int> bpu_core_ids = json_config.value("bpu_core_ids", std::vector<int>{});  // 可选：BPU核配置

  // 3. 初始化 dnn_node 参数
  dnn_node_para_ = std::make_shared<hobot::dnn_node::DnnNodePara>();
  dnn_node_para_->model_file = model_file;                       // 模型文件路径（HBM）
  dnn_node_para_->model_name = model_name;                       // 模型名（多模型时必选）
  dnn_node_para_->task_num = task_num;                           // 并发任务数
  dnn_node_para_->bpu_core_ids = bpu_core_ids;                   // BPU核配置（空则自动分配）
  // 设置模型任务类型：ROI推理 / 普通整图推理
  dnn_node_para_->model_task_type = use_roi_ ? 
    hobot::dnn_node::ModelTaskType::ModelRoiInferType : 
    hobot::dnn_node::ModelTaskType::ModelInferType;

  // 4. 创建 dnn_node 实现实例
  dnn_node_impl_ = std::make_shared<hobot::dnn_node::DnnNodeImpl>(dnn_node_para_);

  // 5. 初始化模型（加载HBM、解析模型信息）
  int ret = dnn_node_impl_->ModelInit();
  if (ret != 0) {
    tools::logger()->error("DnnNode ModelInit failed! ret={}", ret);
    throw std::runtime_error(fmt::format("DnnNode ModelInit failed, ret={}", ret));
  }

  // 6. 初始化推理任务（创建任务池、线程池）
  ret = dnn_node_impl_->TaskInit();
  if (ret != 0) {
    tools::logger()->error("DnnNode TaskInit failed! ret={}", ret);
    throw std::runtime_error(fmt::format("DnnNode TaskInit failed, ret={}", ret));
  }

  // 7. 获取模型实际输入尺寸（覆盖默认值）
  ret = dnn_node_impl_->GetModelInputSize(0, input_w_, input_h_);
  if (ret != 0) {
    tools::logger()->warn("GetModelInputSize failed! Use default size {}x{}", input_w_, input_h_);
  } else {
    tools::logger()->info("Model init success: input_size={}x{}, class_num={}, task_num={}",
                         input_w_, input_h_, class_num_, task_num);
  }
}

// 析构函数：释放 dnn_node 资源（自动管理模型和任务）
YOLO11::~YOLO11()
{
  tools::logger()->info("YOLO11 destroyed. DnnNode resources will be released automatically.");
  // dnn_node_impl_ 智能指针会自动调用析构，释放模型、任务池、线程池
}

// 实现基类的 detect 接口：输入 RDK 原生图像，返回装甲板列表
std::vector<Armor> YOLO11::detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count)
{
  // 1. 校验输入有效性
  if (!hobot_img) {
    tools::logger()->error("detect failed: null RDK image (hobotcv::Image)");
    return {};
  }

  // 2. 校验图像格式（必须是 NV12，与 infer 接口要求一致）
  if (hobot_img->format() != IMAGE_FORMAT_NV12) {
    tools::logger()->error("detect failed: unsupported image format ({}), only NV12 allowed",
                           hobot_img->format());
    return {};
  }

  // 3. 从 RDK 图像中提取 NV12 原始数据、宽高
  const std::uint8_t* nv12_data = static_cast<const std::uint8_t*>(hobot_img->data());
  int img_w = hobot_img->width();
  int img_h = hobot_img->height();

  // 4. 调用已实现的同步 infer 接口，获取检测结果
  auto armors = infer(nv12_data, img_w, img_h);

  // 5. 调试模式：绘制检测结果（适配 RDK 图像）
  if (debug_) {
    // 将 RDK 图像（NV12）转为 cv::Mat 用于绘制
    cv::Mat nv12_mat(img_h * 3 / 2, img_w, CV_8UC1, const_cast<std::uint8_t*>(nv12_data));
    cv::Mat bgr_mat;
    cv::cvtColor(nv12_mat, bgr_mat, cv::COLOR_YUV2BGR_NV12);
    draw_detections(bgr_mat, armors);  // 复用已实现的绘制方法
  }

  // 6. 更新帧计数（用于调试保存）
  frame_count_ = frame_count;

  return armors;
}

// 预处理：NV12 图像转 dnn_node 所需的 DNNInput（NV12PyramidInput）
bool YOLO11::preprocess(const std::uint8_t* nv12_img_data, int img_width, int img_height,
                        std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>& dnn_inputs)
{
  dnn_inputs.clear();
  src_img_w_ = img_width;
  src_img_h_ = img_height;

  // 检查输入有效性
  if (nv12_img_data == nullptr || img_width <= 0 || img_height <= 0) {
    tools::logger()->error("Invalid input: null data or invalid size ({}, {})", img_width, img_height);
    return false;
  }

  // 计算缩放比例和偏移（将原图等比例缩放到模型输入尺寸，居中对齐）
  scale_ = std::min(static_cast<float>(input_w_) / img_width, static_cast<float>(input_h_) / img_height);
  int scaled_w = static_cast<int>(img_width * scale_);
  int scaled_h = static_cast<int>(img_height * scale_);
  offset_.x = (input_w_ - scaled_w) / 2.0f;  // 水平偏移（居中）
  offset_.y = (input_h_ - scaled_h) / 2.0f;  // 垂直偏移（居中）

  // 构造 NV12 图像信息
  hobot::dnn_node::ImageInfo input_img_info;
  input_img_info.data = nv12_img_data;
  input_img_info.width = img_width;
  input_img_info.height = img_height;
  input_img_info.format = hobot::dnn_node::ImageFormat::NV12;  // 输入格式：NV12

  // 创建 NV12 金字塔输入（dnn_node 推荐输入类型，支持自动缩放）
  auto pyramid_input = std::make_shared<hobot::dnn_node::NV12PyramidInput>();
  int ret = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(input_img_info, *pyramid_input);
  if (ret != 0) {
    tools::logger()->error("Preprocess failed: GetNV12PyramidFromNV12Img ret={}", ret);
    return false;
  }

  // 设置模型输入尺寸和缩放模式
  pyramid_input->SetTargetSize(input_w_, input_h_);  // 目标尺寸：模型输入尺寸
  pyramid_input->SetResizeType(hobot::dnn_node::ResizeType::BILINEAR);  // 双线性插值缩放

  dnn_inputs.push_back(pyramid_input);
  tools::logger()->debug("Preprocess success: src_size=({}x{}), scaled_size=({}x{}), offset=({:.1f},{:.1f})",
                         img_width, img_height, scaled_w, scaled_h, offset_.x, offset_.y);
  return true;
}

// 同步推理接口实现
std::vector<Armor> YOLO11::infer(const std::uint8_t* nv12_img_data, int img_width, int img_height,
                               int alloctask_timeout_ms, int infer_timeout_ms)
{
  frame_count_++;

  // 1. 预处理：生成 dnn_node 输入
  std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>> dnn_inputs;
  if (!preprocess(nv12_img_data, img_width, img_height, dnn_inputs)) {
    return {};
  }

  // 2. 创建推理输出对象
  auto dnn_output = std::make_shared<hobot::dnn_node::DnnNodeOutput>();

  // 3. 同步推理（阻塞等待结果）
  int ret = dnn_node_impl_->Run(
    dnn_inputs,                  // DNNInput 类型输入
    {},                          // 无张量输入（tensor_inputs 为空）
    hobot::dnn_node::InputType::DNN_INPUT,  // 输入类型：DNN_INPUT
    dnn_output,                  // 输出存储对象
    std::bind(&YOLO11::postprocess, this, std::placeholders::_1),  // 后处理回调
    nullptr,                     // ROI 列表（非 ROI 推理时为空）
    true,                        // 同步模式
    alloctask_timeout_ms,        // 任务申请超时时间
    infer_timeout_ms             // 推理超时时间
  );

  if (ret != 0) {
    tools::logger()->error("Sync infer failed! ret={}", ret);
    return {};
  }

  // 4. 解析输出结果
  auto armors = parse_output(dnn_output);

  // 5. 调试模式：绘制检测结果并保存
  if (debug_) {
    // NV12 转 BGR（用于绘制）
    cv::Mat nv12_mat(img_height * 3 / 2, img_width, CV_8UC1, const_cast<std::uint8_t*>(nv12_img_data));
    cv::Mat bgr_mat;
    cv::cvtColor(nv12_mat, bgr_mat, cv::COLOR_YUV2BGR_NV12);
    draw_detections(bgr_mat, armors);
  }

  return armors;
}

// 异步推理接口实现
int YOLO11::infer_async(const std::uint8_t* nv12_img_data, int img_width, int img_height,
                        InferCallback callback, int alloctask_timeout_ms, int infer_timeout_ms)
{
  if (!callback) {
    tools::logger()->error("Async infer failed: invalid callback");
    return -1;
  }
  frame_count_++;

  // 1. 预处理：生成 dnn_node 输入
  std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>> dnn_inputs;
  if (!preprocess(nv12_img_data, img_width, img_height, dnn_inputs)) {
    callback({});
    return -1;
  }

  // 2. 拷贝原图（异步模式下避免输入数据生命周期问题）
  cv::Mat nv12_mat(img_height * 3 / 2, img_width, CV_8UC1, const_cast<std::uint8_t*>(nv12_img_data));
  cv::Mat bgr_mat;
  cv::cvtColor(nv12_mat, bgr_mat, cv::COLOR_YUV2BGR_NV12);
  auto img_ptr = std::make_shared<cv::Mat>(bgr_mat.clone());

  // 3. 异步推理（非阻塞，通过回调返回结果）
  int ret = dnn_node_impl_->Run(
    dnn_inputs,
    {},
    hobot::dnn_node::InputType::DNN_INPUT,
    std::make_shared<hobot::dnn_node::DnnNodeOutput>(),
    [this, callback, img_ptr](const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& output) {
      // 后处理：解析结果
      auto armors = this->parse_output(output);
      // 调试绘制
      if (this->debug_) {
        this->draw_detections(*img_ptr, armors);
      }
      // 调用用户回调返回结果
      callback(armors);
    },
    nullptr,
    false,  // 异步模式
    alloctask_timeout_ms,
    infer_timeout_ms
  );

  if (ret != 0) {
    tools::logger()->error("Async infer failed! ret={}", ret);
    callback({});
    return ret;
  }

  return 0;
}

// 后处理回调：dnn_node 推理完成后自动调用
void YOLO11::postprocess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& output,
                         InferCallback callback)
{
  if (!output) {
    tools::logger()->error("Postprocess failed: invalid output");
    if (callback) callback({});
    return;
  }

  // 解析输出并调用回调（同步模式下 callback 为空，仅解析不回调）
  auto armors = parse_output(output);
  if (callback) {
    callback(armors);
  }
}

// 解析输出张量：将 dnn_node 输出转为 Armor 列表
std::vector<Armor> YOLO11::parse_output(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& output)
{
  std::vector<Armor> armors;

  // 检查输出有效性
  if (!output || output->output_tensors.empty()) {
    tools::logger()->error("Parse output failed: no valid tensors");
    return armors;
  }

  // 获取第一个输出张量（YOLO11 通常为单输出张量）
  auto output_tensor = output->output_tensors[0];
  if (!output_tensor || !output_tensor->data) {
    tools::logger()->error("Parse output failed: null tensor data");
    return armors;
  }

  // 解析张量维度（假设格式：[batch, num_boxes, 4+class_num+8]）
  auto& shape = output_tensor->shape;
  int batch_size = shape.dim[0];
  int num_boxes = shape.dim[1];
  int output_dim = shape.dim[2];
  float* output_data = static_cast<float*>(output_tensor->data);

  tools::logger()->debug("Parse output: batch={}, num_boxes={}, dim={}",
                         batch_size, num_boxes, output_dim);

  std::vector<ArmorName> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> bboxes;
  std::vector<std::vector<cv::Point2f>> key_points_list;

  // 遍历所有预测框
  for (int i = 0; i < num_boxes; i++) {
    int box_offset = i * output_dim;
    float* box = output_data + box_offset;

    // 解析边界框（xywh）、类别置信度、关键点
    float x = box[0], y = box[1], w = box[2], h = box[3];
    float* scores = box + 4;
    float* key_points = box + 4 + class_num_;

    // 筛选最大置信度的类别
    float max_score = 0.0f;
    int max_cls_idx = 0;
    for (int cls = 0; cls < class_num_; cls++) {
      if (scores[cls] > max_score) {
        max_score = scores[cls];
        max_cls_idx = cls;
      }
    }

    // 置信度过滤
    if (max_score < min_confidence_) {
      continue;
    }

    // 还原边界框到原图坐标
    int left = static_cast<int>((x - 0.5 * w) / scale_ + offset_.x);
    int top = static_cast<int>((y - 0.5 * h) / scale_ + offset_.y);
    int width = static_cast<int>(w / scale_);
    int height = static_cast<int>(h / scale_);

    // 边界框裁剪到图像范围内
    left = std::max(0, std::min(left, src_img_w_ - 1));
    top = std::max(0, std::min(top, src_img_h_ - 1));
    width = std::max(1, std::min(width, src_img_w_ - left));
    height = std::max(1, std::min(height, src_img_h_ - top));

    // 解析 4 个关键点（还原到原图坐标）
    std::vector<cv::Point2f> key_points_vec;
    for (int kp = 0; kp < 4; kp++) {
      float kp_x = (key_points[kp * 2] / scale_) + offset_.x;
      float kp_y = (key_points[kp * 2 + 1] / scale_) + offset_.y;
      // 关键点裁剪到图像范围内
      kp_x = std::max(0.0f, std::min(kp_x, static_cast<float>(src_img_w_ - 1)));
      kp_y = std::max(0.0f, std::min(kp_y, static_cast<float>(src_img_h_ - 1)));
      key_points_vec.emplace_back(kp_x, kp_y);
    }

    // 保存候选结果
    class_ids.push_back(static_cast<ArmorName>(max_cls_idx));
    confidences.push_back(max_score);
    bboxes.emplace_back(left, top, width, height);
    key_points_list.emplace_back(key_points_vec);
  }

  // NMS 非极大值抑制（去除重叠框）
  std::vector<int> indices;
  cv::dnn::NMSBoxes(bboxes, confidences, min_confidence_, nms_threshold_, indices);

  // 构造最终 Armor 列表
  for (int idx : indices) {
    sort_keypoints(key_points_list[idx]);
    if (use_roi_) {
      cv::Point2f center = cv::Point2f(bboxes[idx].x + bboxes[idx].width / 2.0f, bboxes[idx].y + bboxes[idx].height / 2.0f);
      armors.emplace_back(class_ids[idx], confidences[idx], bboxes[idx], key_points_list[idx], center);
    } else {
      cv::Point2f center = cv::Point2f(bboxes[idx].x + bboxes[idx].width / 2.0f, bboxes[idx].y + bboxes[idx].height / 2.0f);
      armors.emplace_back(class_ids[idx], confidences[idx], bboxes[idx], key_points_list[idx], center);
    }
    armors.back().type = get_type(armors.back());
  }

  tools::logger()->debug("Parse output success: detected {} armors", armors.size());
  return armors;
}

// -------------------------- 原有辅助函数实现 --------------------------
ArmorType YOLO11::get_type(const Armor & armor)
{
  return (armor.class_id == ArmorName::B1 || armor.class_id == ArmorName::R1) ?
         ArmorType::LARGE : ArmorType::SMALL;
}

void YOLO11::draw_detections(const cv::Mat& img, const std::vector<Armor> & armors)
{
  cv::Mat tmp_img = img.clone();

  for (const auto& armor : armors) {
    // 绘制边界框
    cv::rectangle(tmp_img, armor.bbox, cv::Scalar(255, 0, 0), 2);
    // 绘制关键点
    for (const auto& kp : armor.key_points) {
      cv::circle(tmp_img, kp, 3, cv::Scalar(0, 255, 0), -1);
    }
    // 绘制类别和置信度
    std::string label = fmt::format("{} {:.2f}",
      ARMOR_TYPE_STR[static_cast<int>(armor.type)], armor.confidence);
    cv::putText(tmp_img, label, armor.bbox.tl() - cv::Point(0, 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
  }

  // 保存调试图像
  if (!save_path_.empty()) {
    std::filesystem::create_directories(save_path_);
    std::string filename = fmt::format("{}/frame_{:06d}.jpg", save_path_, frame_count_);
    cv::imwrite(filename, tmp_img);
  }

  // 实时显示（可选）
  cv::imshow("YOLO11 Armor Detection", tmp_img);
  cv::waitKey(1);
}

void YOLO11::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  // 按 x 坐标排序（左到右）
  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
    return a.x < b.x;
  });
}

cv::Point2f YOLO11::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return cv::Point2f(center.x / bgr_img.cols, center.y / bgr_img.rows);
}

}  // namespace rm_auto_aim
