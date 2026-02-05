#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/hb_sys.h"

#define MODEL_PATH "/home/sunrise/Inference-for-RDKx5/autoaim/model/yolov5_bgr_NHWC.bin"
#define IMAGE_PATH "/home/sunrise/dataset/000137.jpg"
#define OUTPUT_DIR "/home/sunrise/Inference-for-RDKx5/test_model/output"

#define RDK_CHECK_SUCCESS(value, errmsg)                                         \
  do {                                                                            \
    auto ret_code = value;                                                        \
    if (ret_code != 0) {                                                          \
      std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << std::endl;       \
      std::cout << errmsg << ", error code:" << ret_code << std::endl;            \
      return ret_code;                                                            \
    }                                                                             \
  } while (0)

int main() {
  auto begin_time = std::chrono::system_clock::now();

  hbPackedDNNHandle_t packed_dnn_handle;
  const char *model_file_name = MODEL_PATH;
  RDK_CHECK_SUCCESS(
      hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1),
      "hbDNNInitializeFromFiles failed");

  std::cout << "\033[31m Load model time = " << std::fixed
            << std::setprecision(2)
            << std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::system_clock::now() - begin_time)
                       .count() /
                   1000.0
            << " ms\033[0m" << std::endl;

  const char **model_name_list;
  int model_count = 0;
  RDK_CHECK_SUCCESS(
      hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
      "hbDNNGetModelNameList failed");
  const char *model_name = model_name_list[0];

  hbDNNHandle_t dnn_handle;
  RDK_CHECK_SUCCESS(
      hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name),
      "hbDNNGetModelHandle failed");

  int32_t input_count = 0;
  int32_t output_count = 0;
  RDK_CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnn_handle),
                    "hbDNNGetInputCount failed");
  RDK_CHECK_SUCCESS(hbDNNGetOutputCount(&output_count, dnn_handle),
                    "hbDNNGetOutputCount failed");

  if (input_count != 1) {
    std::cout << "Only support 1 input, got " << input_count << std::endl;
    return -1;
  }

  hbDNNTensor input;
  hbDNNTensorProperties in_props;
  RDK_CHECK_SUCCESS(hbDNNGetInputTensorProperties(&in_props, dnn_handle, 0),
                    "hbDNNGetInputTensorProperties failed");

  if (in_props.tensorType != HB_DNN_IMG_TYPE_BGR ||
      in_props.tensorLayout != HB_DNN_LAYOUT_NHWC ||
      in_props.validShape.numDimensions != 4) {
    std::cout << "Unexpected input format. Expect BGR + NHWC." << std::endl;
    return -1;
  }

  int32_t input_h = in_props.validShape.dimensionSize[1];
  int32_t input_w = in_props.validShape.dimensionSize[2];
  int32_t input_c = in_props.validShape.dimensionSize[3];
  if (input_c != 3) {
    std::cout << "Unexpected input channel: " << input_c << std::endl;
    return -1;
  }

  input.properties = in_props;
  // Use valid shape as aligned shape and allocate alignedByteSize
  input.properties.alignedShape = input.properties.validShape;
  RDK_CHECK_SUCCESS(hbSysAllocCachedMem(&input.sysMem[0],
                                        input.properties.alignedByteSize),
                    "hbSysAllocCachedMem input failed");

  std::vector<hbDNNTensor> output_tensors(output_count);
  for (int i = 0; i < output_count; ++i) {
    hbDNNTensorProperties &out_props = output_tensors[i].properties;
    RDK_CHECK_SUCCESS(hbDNNGetOutputTensorProperties(&out_props, dnn_handle, i),
                      "hbDNNGetOutputTensorProperties failed");
    RDK_CHECK_SUCCESS(
        hbSysAllocCachedMem(&output_tensors[i].sysMem[0],
                             out_props.alignedByteSize),
        "hbSysAllocCachedMem output failed");
  }

  cv::Mat img = cv::imread(IMAGE_PATH, cv::IMREAD_COLOR);
  if (img.empty()) {
    std::cout << "Failed to read image: " << IMAGE_PATH << std::endl;
    return -1;
  }
  cv::resize(img, img, cv::Size(input_w, input_h));

  uint8_t *dst = reinterpret_cast<uint8_t *>(input.sysMem[0].virAddr);
  std::memset(dst, 0, static_cast<size_t>(input.properties.alignedByteSize));

  const int64_t strideN = input.properties.stride[0];
  const int64_t strideH = input.properties.stride[1];
  const int64_t strideW = input.properties.stride[2];
  const int64_t strideC = input.properties.stride[3];

  // NHWC, BGR, uint8
  for (int h = 0; h < input_h; ++h) {
    for (int w = 0; w < input_w; ++w) {
      const int src_idx = (h * input_w + w) * 3;
      const uint8_t b = img.data[src_idx + 0];
      const uint8_t g = img.data[src_idx + 1];
      const uint8_t r = img.data[src_idx + 2];

      const int64_t base = 0 * strideN + h * strideH + w * strideW;
      dst[base + 0 * strideC] = b;
      dst[base + 1 * strideC] = g;
      dst[base + 2 * strideC] = r;
    }
  }

  RDK_CHECK_SUCCESS(hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
                    "hbSysFlushMem input failed");

  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNTensor *out_ptr = output_tensors.data();
  RDK_CHECK_SUCCESS(
      hbDNNInfer(&task_handle, &out_ptr, &input, dnn_handle, &infer_ctrl_param),
      "hbDNNInfer failed");
  RDK_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                    "hbDNNWaitTaskDone failed");

  for (int i = 0; i < output_count; ++i) {
    RDK_CHECK_SUCCESS(
        hbSysFlushMem(&output_tensors[i].sysMem[0],
                       HB_SYS_MEM_CACHE_INVALIDATE),
        "hbSysFlushMem output failed");

    auto &shape = output_tensors[i].properties.validShape;
    std::cout << "[output " << i << "] numDim=" << shape.numDimensions;
    for (int d = 0; d < shape.numDimensions; ++d) {
      std::cout << " dim" << d << "=" << shape.dimensionSize[d];
    }
    std::cout << std::endl;
  }

  // Postprocess (OpenvinoInfer-style)
  if (output_count > 0) {
    hbDNNTensor &pp_output = output_tensors[0];
    auto &vshape = pp_output.properties.validShape;
    auto qtype = pp_output.properties.quantiType;

    int nd = vshape.numDimensions;
    int rows = 0;
    int cols = 0;
    if (nd == 4) {
      rows = static_cast<int>(vshape.dimensionSize[1]);
      cols = static_cast<int>(vshape.dimensionSize[2]);
    } else if (nd == 3) {
      rows = static_cast<int>(vshape.dimensionSize[1]);
      cols = static_cast<int>(vshape.dimensionSize[2]);
    } else if (nd == 2) {
      rows = static_cast<int>(vshape.dimensionSize[0]);
      cols = static_cast<int>(vshape.dimensionSize[1]);
    }
    if (cols != 22) {
      int64_t total = 1;
      for (int i = 0; i < nd; ++i) total *= vshape.dimensionSize[i];
      if (total % 22 == 0) {
        cols = 22;
        rows = static_cast<int>(total / 22);
      }
    }

    std::vector<float> out;
    float *data = nullptr;
    int64_t total_elems = static_cast<int64_t>(rows) * cols;
    if (qtype == SHIFT) {
      out.resize(total_elems);
      const int8_t *src = reinterpret_cast<int8_t *>(pp_output.sysMem[0].virAddr);
      int shift = (pp_output.properties.shift.shiftData ? pp_output.properties.shift.shiftData[0] : 0);
      float scale = 1.0f / static_cast<float>(1 << shift);
      for (int64_t i = 0; i < total_elems; ++i) out[i] = src[i] * scale;
      data = out.data();
    } else if (qtype == SCALE) {
      out.resize(total_elems);
      const int8_t *src = reinterpret_cast<int8_t *>(pp_output.sysMem[0].virAddr);
      float scale = (pp_output.properties.scale.scaleData ? pp_output.properties.scale.scaleData[0] : 1.0f);
      for (int64_t i = 0; i < total_elems; ++i) out[i] = src[i] * scale;
      data = out.data();
    } else {
      data = reinterpret_cast<float *>(pp_output.sysMem[0].virAddr);
    }

    auto sigmoid = [](float x) { return 1.0f / (1.0f + std::exp(-x)); };
    float conf_threshold = 0.65f;
    float nms_threshold = 0.45f;
    int detect_color = -1;

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    for (int i = 0; i < rows; ++i) {
      const float *r = data + i * cols;
      float confidence = sigmoid(r[8]);
      if (confidence < conf_threshold) continue;

      int color_id = 0;
      float color_max = r[9];
      for (int k = 10; k <= 12; ++k) {
        if (r[k] > color_max) { color_max = r[k]; color_id = k - 9; }
      }
      if (color_id == 2 || color_id == 3) continue;
      if (detect_color == 0 && color_id == 1) continue;
      if (detect_color == 1 && color_id == 0) continue;

      int cls_id = 0;
      float cls_max = r[13];
      for (int k = 14; k <= 21; ++k) {
        if (r[k] > cls_max) { cls_max = r[k]; cls_id = k - 13; }
      }

      float x0 = r[0], y0 = r[1], x1 = r[2], y1 = r[3];
      float x2 = r[4], y2 = r[5], x3 = r[6], y3 = r[7];
      float minx = std::min(std::min(x0, x1), std::min(x2, x3));
      float maxx = std::max(std::max(x0, x1), std::max(x2, x3));
      float miny = std::min(std::min(y0, y1), std::min(y2, y3));
      float maxy = std::max(std::max(y0, y1), std::max(y2, y3));
      cv::Rect rect(minx, miny, std::max(0.0f, maxx - minx), std::max(0.0f, maxy - miny));
      if (rect.width <= 0 || rect.height <= 0) continue;

      boxes.push_back(rect);
      confidences.push_back(cls_max);
      class_ids.push_back(cls_id);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
    std::cout << "detections: " << indices.size() << std::endl;

    // Draw and save
    for (int idx : indices) {
      if (idx < 0 || idx >= static_cast<int>(boxes.size())) continue;
      cv::rectangle(img, boxes[idx], cv::Scalar(0, 255, 0), 2);
      std::string text = std::to_string(class_ids[idx]) + ":" +
                         cv::format("%.2f", confidences[idx]);
      cv::Point origin(boxes[idx].x, std::max(0, boxes[idx].y - 5));
      cv::putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    std::string base = IMAGE_PATH;
    auto pos = base.find_last_of("/\\");
    std::string file = (pos == std::string::npos) ? base : base.substr(pos + 1);
    std::string out_dir = OUTPUT_DIR;
    if (!out_dir.empty() && out_dir.back() != '/' && out_dir.back() != '\\') {
      out_dir += "/";
    }
    std::string out_path = out_dir + file;
    cv::imwrite(out_path, img);
    std::cout << "saved: " << out_path << std::endl;
  }

  RDK_CHECK_SUCCESS(hbDNNReleaseTask(task_handle),
                    "hbDNNReleaseTask failed");
  RDK_CHECK_SUCCESS(hbSysFreeMem(&(input.sysMem[0])),
                    "hbSysFreeMem input failed");
  for (int i = 0; i < output_count; ++i) {
    RDK_CHECK_SUCCESS(hbSysFreeMem(&(output_tensors[i].sysMem[0])),
                      "hbSysFreeMem output failed");
  }
  RDK_CHECK_SUCCESS(hbDNNRelease(packed_dnn_handle),
                    "hbDNNRelease failed");

  return 0;
}
