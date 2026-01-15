// Copyright (C) 2026
// Licensed under the MIT License.

#include "armor_detector/yolo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/hb_sys.h"

namespace rm_auto_aim
{

static float sigmoid(float x)
{
  return 1.0f / (1.0f + std::exp(-x));
}

static void hbCheck(int ret, const char * what)
{
  if (ret != 0) {
    throw std::runtime_error(std::string(what) + ", ret=" + std::to_string(ret));
  }
}

static std::vector<std::string> buildClassNames()
{
  return {"G", "1", "2", "3", "4", "5", "O", "Bs", "Bb"};
}

const std::vector<std::string> & Yolo::classNames()
{
  static const std::vector<std::string> kNames = buildClassNames();
  return kNames;
}

struct Yolo::Impl
{
  Params params;

  hbPackedDNNHandle_t packed = nullptr;
  hbDNNHandle_t model = nullptr;

  hbDNNTensor input;
  std::vector<hbDNNTensor> outputs;

  int input_h = 0;
  int input_w = 0;
  int output_count = 0;

  Impl(const Params & p) : params(p)
  {
    if (params.model_path.empty()) {
      throw std::invalid_argument("yolo model_path is empty");
    }

    const char * model_file_name = params.model_path.c_str();
    hbCheck(hbDNNInitializeFromFiles(&packed, &model_file_name, 1), "hbDNNInitializeFromFiles");

    const char ** model_name_list = nullptr;
    int model_count = 0;
    hbCheck(hbDNNGetModelNameList(&model_name_list, &model_count, packed), "hbDNNGetModelNameList");
    if (model_count <= 0) {
      throw std::runtime_error("No model in packed bin");
    }

    hbCheck(hbDNNGetModelHandle(&model, packed, model_name_list[0]), "hbDNNGetModelHandle");

    int32_t input_count = 0;
    hbCheck(hbDNNGetInputCount(&input_count, model), "hbDNNGetInputCount");
    if (input_count != 1) {
      throw std::runtime_error("Expect 1 input");
    }

    hbDNNTensorProperties input_props;
    hbCheck(hbDNNGetInputTensorProperties(&input_props, model, 0), "hbDNNGetInputTensorProperties");
    if (input_props.tensorLayout != HB_DNN_LAYOUT_NCHW) {
      throw std::runtime_error("Expect NCHW input");
    }
    if (input_props.validShape.numDimensions != 4) {
      throw std::runtime_error("Expect 4D input (NCHW)");
    }

    input_h = static_cast<int>(input_props.validShape.dimensionSize[2]);
    input_w = static_cast<int>(input_props.validShape.dimensionSize[3]);

    input.properties = input_props;
    hbCheck(hbSysAllocCachedMem(&input.sysMem[0], int(3 * input_h * input_w)), "hbSysAllocCachedMem(input)");

    int32_t out_count = 0;
    hbCheck(hbDNNGetOutputCount(&out_count, model), "hbDNNGetOutputCount");
    output_count = out_count;
    if (output_count <= 0) {
      throw std::runtime_error("No outputs");
    }

    outputs.resize(output_count);
    for (int i = 0; i < output_count; ++i) {
      hbDNNTensorProperties & out_props = outputs[i].properties;
      hbCheck(hbDNNGetOutputTensorProperties(&out_props, model, i), "hbDNNGetOutputTensorProperties");
      hbSysMem & mem = outputs[i].sysMem[0];
      hbCheck(hbSysAllocCachedMem(&mem, out_props.alignedByteSize), "hbSysAllocCachedMem(output)");
    }
  }

  ~Impl()
  {
    for (auto & t : outputs) {
      hbSysFreeMem(&t.sysMem[0]);
    }
    hbSysFreeMem(&input.sysMem[0]);

    if (packed) {
      hbDNNRelease(packed);
      packed = nullptr;
    }
  }

  static void letterboxRGB(
    const cv::Mat & rgb, int dst_w, int dst_h, cv::Mat & out,
    float & scale, int & x_shift, int & y_shift)
  {
    if (rgb.empty() || rgb.type() != CV_8UC3) {
      throw std::invalid_argument("infer expects CV_8UC3 RGB image");
    }

    scale = std::min(1.0f * dst_h / rgb.rows, 1.0f * dst_w / rgb.cols);
    if (scale <= 0.0f) {
      throw std::runtime_error("Invalid scale");
    }

    int new_w = static_cast<int>(std::round(rgb.cols * scale));
    int new_h = static_cast<int>(std::round(rgb.rows * scale));

    x_shift = (dst_w - new_w) / 2;
    y_shift = (dst_h - new_h) / 2;

    int x_other = dst_w - new_w - x_shift;
    int y_other = dst_h - new_h - y_shift;

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(new_w, new_h));
    cv::copyMakeBorder(
      resized, out, y_shift, y_other, x_shift, x_other,
      cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));
  }

  void fillInputNCHW_RGB_S8Minus128(const cv::Mat & rgb_letterboxed)
  {
    uint8_t * data_u8 = const_cast<uint8_t *>(rgb_letterboxed.ptr<uint8_t>());
    int8_t * data_s8 = reinterpret_cast<int8_t *>(input.sysMem[0].virAddr);

    for (int h = 0; h < input_h; ++h) {
      for (int w = 0; w < input_w; ++w) {
        const uint8_t r = data_u8[h * input_w * 3 + w * 3 + 0];
        const uint8_t g = data_u8[h * input_w * 3 + w * 3 + 1];
        const uint8_t b = data_u8[h * input_w * 3 + w * 3 + 2];
        data_s8[(0 * input_h * input_w) + h * input_w + w] = static_cast<int8_t>(r - 128);
        data_s8[(1 * input_h * input_w) + h * input_w + w] = static_cast<int8_t>(g - 128);
        data_s8[(2 * input_h * input_w) + h * input_w + w] = static_cast<int8_t>(b - 128);
      }
    }
  }

  std::vector<Yolo::Detection> postprocess(
    const cv::Mat & orig_rgb,
    const hbDNNTensor & out_tensor,
    float scale, int x_shift, int y_shift)
  {
    hbSysFlushMem(const_cast<hbSysMem *>(&out_tensor.sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

    const auto & vshape = out_tensor.properties.validShape;
    const auto qtype = out_tensor.properties.quantiType;

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
    } else {
      throw std::runtime_error("Unsupported output dims");
    }

    if (cols != 22) {
      int64_t total = 1;
      for (int i = 0; i < nd; ++i) {
        total *= vshape.dimensionSize[i];
      }
      if (total % 22 == 0) {
        cols = 22;
        rows = static_cast<int>(total / 22);
      }
    }
    if (cols != 22) {
      throw std::runtime_error("Expect 22 columns output");
    }

    int64_t total_elems = static_cast<int64_t>(rows) * static_cast<int64_t>(cols);

    float * data = nullptr;
    std::vector<float> deq;

    if (qtype == SHIFT) {
      deq.resize(total_elems);
      const int8_t * src = reinterpret_cast<int8_t *>(out_tensor.sysMem[0].virAddr);
      int shift = (out_tensor.properties.shift.shiftData ? out_tensor.properties.shift.shiftData[0] : 0);
      float s = 1.0f / static_cast<float>(1 << shift);
      for (int64_t i = 0; i < total_elems; ++i) {
        deq[i] = src[i] * s;
      }
      data = deq.data();
    } else if (qtype == SCALE) {
      deq.resize(total_elems);
      const int8_t * src = reinterpret_cast<int8_t *>(out_tensor.sysMem[0].virAddr);
      float s = (out_tensor.properties.scale.scaleData ? out_tensor.properties.scale.scaleData[0] : 1.0f);
      for (int64_t i = 0; i < total_elems; ++i) {
        deq[i] = src[i] * s;
      }
      data = deq.data();
    } else {
      data = reinterpret_cast<float *>(out_tensor.sysMem[0].virAddr);
    }

    const float conf_thr = params.score_threshold;
    const float nms_thr = params.nms_threshold;

    const float ratio = (scale > 0.0f ? 1.0f / scale : 1.0f);
    const int original_w = orig_rgb.cols;
    const int original_h = orig_rgb.rows;
    const float x_offset = static_cast<float>(x_shift);
    const float y_offset = static_cast<float>(y_shift);

    auto scale_x = [&](float value) {
      float scaled = (value - x_offset) * ratio;
      if (original_w > 0) {
        scaled = std::min(std::max(scaled, 0.0f), static_cast<float>(original_w - 1));
      }
      return scaled;
    };
    auto scale_y = [&](float value) {
      float scaled = (value - y_offset) * ratio;
      if (original_h > 0) {
        scaled = std::min(std::max(scaled, 0.0f), static_cast<float>(original_h - 1));
      }
      return scaled;
    };

    struct Candidate
    {
      cv::Rect box;
      float score;
      Yolo::Detection det;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(rows);
    std::vector<cv::Rect> nms_boxes;
    std::vector<float> nms_scores;

    for (int i = 0; i < rows; ++i) {
      const float * r = data + i * cols;

      float obj = sigmoid(r[8]);
      if (obj < conf_thr) {
        continue;
      }

      // color logits at [9..12]
      int color_idx = 0;
      float color_max = r[9];
      for (int k = 10; k <= 12; ++k) {
        if (r[k] > color_max) {
          color_max = r[k];
          color_idx = k - 9;
        }
      }

      // Drop gray/purple
      if (color_idx >= 2) {
        continue;
      }
      if (params.detect_color == 0 && color_idx == 1) {
        continue;
      }
      if (params.detect_color == 1 && color_idx == 0) {
        continue;
      }

      // class logits at [13..21]
      int cls_idx = 0;
      float cls_max = r[13];
      for (int k = 14; k <= 21; ++k) {
        if (r[k] > cls_max) {
          cls_max = r[k];
          cls_idx = k - 13;
        }
      }

      // keypoints on resized input: TL,BL,BR,TR
      float tlx = scale_x(r[0]);
      float tly = scale_y(r[1]);
      float blx = scale_x(r[2]);
      float bly = scale_y(r[3]);
      float brx = scale_x(r[4]);
      float bry = scale_y(r[5]);
      float trx = scale_x(r[6]);
      float try_ = scale_y(r[7]);

      float minx = std::min(std::min(tlx, blx), std::min(brx, trx));
      float maxx = std::max(std::max(tlx, blx), std::max(brx, trx));
      float miny = std::min(std::min(tly, bly), std::min(bry, try_));
      float maxy = std::max(std::max(tly, bly), std::max(bry, try_));

      cv::Rect rect(
        static_cast<int>(minx), static_cast<int>(miny),
        static_cast<int>(std::max(0.0f, maxx - minx)),
        static_cast<int>(std::max(0.0f, maxy - miny)));
      if (rect.width <= 0 || rect.height <= 0) {
        continue;
      }

      float score = obj * cls_max;
      if (score < conf_thr) {
        continue;
      }

      Yolo::Detection det;
      det.kpts = {cv::Point2f(tlx, tly), cv::Point2f(blx, bly), cv::Point2f(brx, bry), cv::Point2f(trx, try_)};
      det.color_idx = color_idx;
      det.cls_idx = cls_idx;
      det.obj = obj;
      det.cls_score = cls_max;
      det.score = score;

      candidates.push_back({rect, score, det});
      nms_boxes.push_back(rect);
      nms_scores.push_back(score);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(nms_boxes, nms_scores, conf_thr, nms_thr, indices, 1.0f, params.nms_top_k);

    std::vector<Yolo::Detection> kept;
    kept.reserve(indices.size());
    for (int idx : indices) {
      if (idx >= 0 && idx < static_cast<int>(candidates.size())) {
        kept.push_back(candidates[idx].det);
      }
    }

    return kept;
  }
};

Yolo::Yolo(const Params & params) : impl_(std::make_unique<Impl>(params))
{
}

Yolo::~Yolo() = default;

std::vector<Yolo::Detection> Yolo::infer(const cv::Mat & rgb)
{
  float scale = 1.0f;
  int x_shift = 0;
  int y_shift = 0;

  cv::Mat letterboxed;
  Impl::letterboxRGB(rgb, impl_->input_w, impl_->input_h, letterboxed, scale, x_shift, y_shift);

  impl_->fillInputNCHW_RGB_S8Minus128(letterboxed);

  hbSysFlushMem(&impl_->input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

  hbDNNTaskHandle_t task_handle = nullptr;

  hbDNNTensor * out_ptrs = impl_->outputs.data();
  hbCheck(
    hbDNNInfer(&task_handle, &out_ptrs, &impl_->input, impl_->model, &infer_ctrl_param),
    "hbDNNInfer");
  hbCheck(hbDNNWaitTaskDone(task_handle, 0), "hbDNNWaitTaskDone");

  // Expect output[0] contains the proposals with 22 cols
  auto dets = impl_->postprocess(rgb, impl_->outputs.at(0), scale, x_shift, y_shift);

  hbDNNReleaseTask(task_handle);
  return dets;
}

}  // namespace rm_auto_aim
