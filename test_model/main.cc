/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

Copyright (c) 2024，WuChao D-Robotics.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// 注意: 此程序在RDK板端端运行
// Attention: This program runs on RDK board.

// D-Robotics *.bin 模型路径
// Path of D-Robotics *.bin model.
#define MODEL_PATH "../../ptq_models/YOLO11s_detect_bayese_640x640_nchwrgb_modified.bin"

// 推理使用的测试图片路径
// Path of the test image used for inference.
#define TESR_IMG_PATH "../../../../../../resource/datasets/COCO2017/assets/bus.jpg"
// #define TESR_IMG_PATH "../../../../datasets/COCO2017/assets/bus.jpg"

// 前处理方式选择, 0:Resize, 1:LetterBox
// Preprocessing method selection, 0: Resize, 1: LetterBox
#define RESIZE_TYPE 0
#define LETTERBOX_TYPE 1
#define PREPROCESS_TYPE LETTERBOX_TYPE

// 推理结果保存路径
// Path where the inference result will be saved
#define IMG_SAVE_PATH "cpp_result.jpg"

// 模型的类别数量, 默认80
// Number of classes in the model, default is 80
#define CLASSES_NUM 80

// NMS的阈值, 默认0.45
// Non-Maximum Suppression (NMS) threshold, default is 0.45
#define NMS_THRESHOLD 0.7

// 分数阈值, 默认0.25
// Score threshold, default is 0.25
#define SCORE_THRESHOLD 0.25

// NMS选取的前K个框数, 默认300
// Number of top-K boxes selected by NMS, default is 300
#define NMS_TOP_K 300

// C/C++ Standard Librarys
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <string>

// Thrid Party Librarys
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

// RDK BPU libDNN API
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/plugin/hb_dnn_layer.h"
#include "dnn/plugin/hb_dnn_plugin.h"
#include "dnn/hb_sys.h"

#define RDK_CHECK_SUCCESS(value, errmsg)                                         \
    do                                                                           \
    {                                                                            \
        auto ret_code = value;                                                   \
        if (ret_code != 0)                                                       \
        {                                                                        \
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::cout << errmsg << ", error code:" << ret_code << std::endl;     \
            return ret_code;                                                     \
        }                                                                        \
    } while (0);

// COCO Names
std::vector<std::string> object_names = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

int main()
{
    // 0. 加载bin模型
    auto begin_time = std::chrono::system_clock::now();

    hbPackedDNNHandle_t packed_dnn_handle;
    const char *model_file_name = MODEL_PATH;
    RDK_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1),
        "hbDNNInitializeFromFiles failed");

    std::cout << "\033[31m Load D-Robotics Quantize model time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;

    // 1. 打印相关版本信息
    std::cout << "[INFO] OpenCV Version: " << CV_VERSION << std::endl;
    std::cout << "[INFO] MODEL_PATH: " << MODEL_PATH << std::endl;
    std::cout << "[INFO] CLASSES_NUM: " << CLASSES_NUM << std::endl;
    std::cout << "[INFO] NMS_THRESHOLD: " << NMS_THRESHOLD << std::endl;
    std::cout << "[INFO] SCORE_THRESHOLD: " << SCORE_THRESHOLD << std::endl;

    // 2. 打印模型信息
    const char **model_name_list;
    int model_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
        "hbDNNGetModelNameList failed");

    if (model_count > 1)
    {
        std::cout << "This model file have more than 1 model, only use model 0.";
    }
    const char *model_name = model_name_list[0];
    std::cout << "[model name]: " << model_name << std::endl;

    hbDNNHandle_t dnn_handle;
    RDK_CHECK_SUCCESS(
        hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name),
        "hbDNNGetModelHandle failed");

    int32_t input_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputCount(&input_count, dnn_handle),
        "hbDNNGetInputCount failed");

    hbDNNTensorProperties input_properties;
    RDK_CHECK_SUCCESS(
        hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0),
        "hbDNNGetInputTensorProperties failed");

    if (input_count > 1)
    {
        std::cout << "Your Model have more than 1 input, please check!" << std::endl;
        return -1;
    }

    if (input_properties.tensorType == HB_DNN_IMG_TYPE_RGB)
    {
        std::cout << "input tensor type: HB_DNN_IMG_TYPE_RGB" << std::endl;
    }
    else
    {
        std::cout << "input tensor type is not HB_DNN_IMG_TYPE_RGB, please check!" << std::endl;
        return -1;
    }

    std::cout << "input_properties.tensorType: " << input_properties.tensorType << std::endl;
    if (input_properties.tensorLayout == HB_DNN_LAYOUT_NCHW)
    {
        std::cout << "input tensor layout: HB_DNN_LAYOUT_NCHW" << std::endl;
    }
    else
    {
        std::cout << "input tensor layout is not HB_DNN_LAYOUT_NCHW, please check!" << std::endl;
        return -1;
    }

    int32_t input_H, input_W;
    if (input_properties.validShape.numDimensions == 4)
    {
        input_H = input_properties.validShape.dimensionSize[2];
        input_W = input_properties.validShape.dimensionSize[3];
        std::cout << "input tensor valid shape: (" << input_properties.validShape.dimensionSize[0];
        std::cout << ", " << input_properties.validShape.dimensionSize[1];
        std::cout << ", " << input_H;
        std::cout << ", " << input_W << ")" << std::endl;
    }
    else
    {
        std::cout << "input tensor validShape.numDimensions is not 4 such as (1,3,640,640), please check!" << std::endl;
        return -1;
    }

    int32_t output_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetOutputCount(&output_count, dnn_handle),
        "hbDNNGetOutputCount failed");

    if (output_count == 6)
    {
        for (int i = 0; i < 6; i++)
        {
            hbDNNTensorProperties output_properties;
            RDK_CHECK_SUCCESS(
                hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i),
                "hbDNNGetOutputTensorProperties failed");
            std::cout << "output[" << i << "] ";
            std::cout << "valid shape: (" << output_properties.validShape.dimensionSize[0];
            std::cout << ", " << output_properties.validShape.dimensionSize[1];
            std::cout << ", " << output_properties.validShape.dimensionSize[2];
            std::cout << ", " << output_properties.validShape.dimensionSize[3] << "), ";
            if (output_properties.quantiType == SHIFT)
                std::cout << "quantiType: SHIFT" << std::endl;
            if (output_properties.quantiType == SCALE)
                std::cout << "quantiType: SCALE" << std::endl;
            if (output_properties.quantiType == NONE)
                std::cout << "quantiType: NONE" << std::endl;
        }
    }
    else
    {
        std::cout << "Your Model's outputs num is not 6, please check!" << std::endl;
        return -1;
    }

    int order[6] = {0, 1, 2, 3, 4, 5};
    int32_t H_8 = input_H / 8;
    int32_t H_16 = input_H / 16;
    int32_t H_32 = input_H / 32;
    int32_t W_8 = input_W / 8;
    int32_t W_16 = input_W / 16;
    int32_t W_32 = input_W / 32;
    int32_t order_we_want[6][3] = {
        {H_8, W_8, CLASSES_NUM},   // output[order[3]]: (1, H // 8,  W // 8,  CLASSES_NUM)
        {H_8, W_8, 64},            // output[order[0]]: (1, H // 8,  W // 8,  64)
        {H_16, W_16, CLASSES_NUM}, // output[order[4]]: (1, H // 16, W // 16, CLASSES_NUM)
        {H_16, W_16, 64},          // output[order[1]]: (1, H // 16, W // 16, 64)
        {H_32, W_32, CLASSES_NUM}, // output[order[5]]: (1, H // 32, W // 32, CLASSES_NUM)
        {H_32, W_32, 64},          // output[order[2]]: (1, H // 32, W // 32, 64)
    };
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            hbDNNTensorProperties output_properties;
            RDK_CHECK_SUCCESS(
                hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, j),
                "hbDNNGetOutputTensorProperties failed");
            int32_t h = output_properties.validShape.dimensionSize[1];
            int32_t w = output_properties.validShape.dimensionSize[2];
            int32_t c = output_properties.validShape.dimensionSize[3];
            if (h == order_we_want[i][0] && w == order_we_want[i][1] && c == order_we_want[i][2])
            {
                order[i] = j;
                break;
            }
        }
    }

    if (order[0] + order[1] + order[2] + order[3] + order[4] + order[5] == 0 + 1 + 2 + 3 + 4 + 5)
    {
        std::cout << "Outputs order check SUCCESS, continue." << std::endl;
        std::cout << "order = {";
        for (int i = 0; i < 6; i++)
        {
            std::cout << order[i] << ", ";
        }
        std::cout << "}" << std::endl;
    }
    else
    {
        std::cout << "Outputs order check FAILED, use default" << std::endl;
        for (int i = 0; i < 6; i++)
            order[i] = i;
    }

    // 3. 利用OpenCV准备nv12的输入数据
    float y_scale = 1.0;
    float x_scale = 1.0;
    int x_shift = 0;
    int y_shift = 0;

    cv::Mat img = cv::imread(TESR_IMG_PATH);
    std::cout << "img path: " << TESR_IMG_PATH << std::endl;
    std::cout << "img (cols, rows, channels): (";
    std::cout << img.rows << ", ";
    std::cout << img.cols << ", ";
    std::cout << img.channels() << ")" << std::endl;

    cv::Mat resize_img;
    if (PREPROCESS_TYPE == LETTERBOX_TYPE)
    {
        begin_time = std::chrono::system_clock::now();
        x_scale = std::min(1.0 * input_H / img.rows, 1.0 * input_W / img.cols);
        y_scale = x_scale;
        if (x_scale <= 0 || y_scale <= 0)
        {
            throw std::runtime_error("Invalid scale factor.");
        }

        int new_w = img.cols * x_scale;
        x_shift = (input_W - new_w) / 2;
        int x_other = input_W - new_w - x_shift;

        int new_h = img.rows * y_scale;
        y_shift = (input_H - new_h) / 2;
        int y_other = input_H - new_h - y_shift;

        cv::Size targetSize(new_w, new_h);
        cv::resize(img, resize_img, targetSize);
        cv::copyMakeBorder(resize_img, resize_img, y_shift, y_other, x_shift, x_other, cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));

        std::cout << "\033[31m pre process (LetterBox) time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;
    }
    else if (PREPROCESS_TYPE == RESIZE_TYPE)
    {
        begin_time = std::chrono::system_clock::now();

        cv::Size targetSize(input_W, input_H);
        cv::resize(img, resize_img, targetSize);

        y_scale = 1.0 * input_H / img.rows;
        x_scale = 1.0 * input_W / img.cols;
        y_shift = 0;
        x_shift = 0;

        std::cout << "\033[31m pre process (Resize) time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;
    }
    std::cout << "y_scale = " << y_scale << ", ";
    std::cout << "x_scale = " << x_scale << std::endl;
    std::cout << "y_shift = " << y_shift << ", ";
    std::cout << "x_shift = " << x_shift << std::endl;

    hbDNNTensor input;
    input.properties = input_properties;
    hbSysAllocCachedMem(&input.sysMem[0], int(3 * input_H * input_W));

    begin_time = std::chrono::system_clock::now();
    uint8_t *data_u8{reinterpret_cast<uint8_t *>(resize_img.ptr<uint8_t>())};
    int8_t *data_s8{reinterpret_cast<int8_t *>(input.sysMem[0].virAddr)};

    for (int h = 0; h < input_H; h++)
    {
        for (int w = 0; w < input_W; w++)
        {
            data_s8[(0 * input_H * input_W) + h * input_W + w] = static_cast<int8_t>(data_u8[h * input_W * 3 + w * 3 + 2] - 128); // R
            data_s8[(1 * input_H * input_W) + h * input_W + w] = static_cast<int8_t>(data_u8[h * input_W * 3 + w * 3 + 1] - 128); // G
            data_s8[(2 * input_H * input_W) + h * input_W + w] = static_cast<int8_t>(data_u8[h * input_W * 3 + w * 3 + 0] - 128); // B
        }
    }

    std::cout << "\033[31m (u8-128)->s8 time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;

    begin_time = std::chrono::system_clock::now();

    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

    hbDNNTensor *output = new hbDNNTensor[output_count];
    for (int i = 0; i < output_count; i++)
    {
        hbDNNTensorProperties &output_properties = output[i].properties;
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
        int out_aligned_size = output_properties.alignedByteSize;
        hbSysMem &mem = output[i].sysMem[0];
        hbSysAllocCachedMem(&mem, out_aligned_size);
    }

    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param);

    hbDNNWaitTaskDone(task_handle, 0);
    std::cout << "\033[31m forward time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;

    // 7. YOLO 后处理（移植自 armor_detector/yolo.cpp）
    begin_time = std::chrono::system_clock::now();

    const float conf_thr = SCORE_THRESHOLD;
    const float nms_thr = NMS_THRESHOLD;
    const int detect_color = -1; // -1 保留红/蓝，丢弃灰/紫

    hbDNNTensor &pp_output = output[0];
    hbSysFlushMem(&(pp_output.sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

    auto &vshape = pp_output.properties.validShape;
    int nd = vshape.numDimensions;
    int rows = 0;
    int cols = 0;
    if (nd == 3)
    {
        rows = static_cast<int>(vshape.dimensionSize[1]);
        cols = static_cast<int>(vshape.dimensionSize[2]);
    }
    else if (nd == 2)
    {
        rows = static_cast<int>(vshape.dimensionSize[0]);
        cols = static_cast<int>(vshape.dimensionSize[1]);
    }
    else if (nd == 4)
    {
        rows = static_cast<int>(vshape.dimensionSize[2]);
        cols = static_cast<int>(vshape.dimensionSize[3]);
    }
    else
    {
        std::cout << "Unsupported tensor dims: " << nd << std::endl;
        return -1;
    }

    if (cols < 22)
    {
        std::cout << "Expect >=22 columns, got " << cols << std::endl;
        return -1;
    }

    float *data = reinterpret_cast<float *>(pp_output.sysMem[0].virAddr);
    auto sigmoid = [](float x)
    { return 1.0f / (1.0f + std::exp(-x)); };

    float ratio = (x_scale > 0 ? 1.0f / x_scale : 1.0f);
    int original_w = img.cols;
    int original_h = img.rows;
    float x_offset = static_cast<float>(x_shift);
    float y_offset = static_cast<float>(y_shift);
    auto scale_x = [&](float value)
    {
        float scaled = (value - x_offset) * ratio;
        if (original_w > 0)
        {
            scaled = std::min(std::max(scaled, 0.0f), static_cast<float>(original_w - 1));
        }
        return scaled;
    };
    auto scale_y = [&](float value)
    {
        float scaled = (value - y_offset) * ratio;
        if (original_h > 0)
        {
            scaled = std::min(std::max(scaled, 0.0f), static_cast<float>(original_h - 1));
        }
        return scaled;
    };

    struct Detection
    {
        cv::Rect2f box;
        float class_score;
        float disp_score;
        std::string class_name;
        std::vector<cv::Point2f> kpts;
    };

    std::vector<Detection> dets;
    dets.reserve(rows);

    for (int i = 0; i < rows; ++i)
    {
        const float *r = data + i * cols;
        float obj = sigmoid(r[8]);
        if (obj < conf_thr)
            continue;

        int color_idx = 0;
        float color_max = r[9];
        for (int k = 10; k <= 12; ++k)
        {
            if (r[k] > color_max)
            {
                color_max = r[k];
                color_idx = k - 9;
            }
        }
        if (color_idx >= 2)
            continue; // 丢弃灰/紫
        if (detect_color == 0 && color_idx == 1)
            continue; // 只要红
        if (detect_color == 1 && color_idx == 0)
            continue; // 只要蓝

        int cls_idx = 0;
        float cls_max = r[13];
        for (int k = 14; k <= 21; ++k)
        {
            if (r[k] > cls_max)
            {
                cls_max = r[k];
                cls_idx = k - 13;
            }
        }

        float tlx = scale_x(r[0]);
        float tly = scale_y(r[1]);
        float blx = scale_x(r[2]);
        float bly = scale_y(r[3]);
        float brx = scale_x(r[4]);
        float bry = scale_y(r[5]);
        float trx = scale_x(r[6]);
        float try_ = scale_y(r[7]);

        std::vector<cv::Point2f> kps{{blx, bly}, {tlx, tly}, {trx, try_}, {brx, bry}};

        float minx = std::min(std::min(tlx, blx), std::min(brx, trx));
        float maxx = std::max(std::max(tlx, blx), std::max(brx, trx));
        float miny = std::min(std::min(tly, bly), std::min(bry, try_));
        float maxy = std::max(std::max(tly, bly), std::max(bry, try_));
        cv::Rect2f rect(minx, miny, std::max(0.0f, maxx - minx), std::max(0.0f, maxy - miny));

        static const char *num_labels[9] = {"G", "1", "2", "3", "4", "5", "O", "Bs", "Bb"};
        std::string label = num_labels[cls_idx];
        std::string class_name;
        if (label == "1" || label == "2" || label == "3" || label == "4" || label == "5")
        {
            class_name = (color_idx == 0 ? "R" : "B");
            class_name += label;
        }
        else
        {
            class_name = label;
        }

        float class_score = cls_max;
        float final_score = obj * cls_max;

        dets.push_back({rect, class_score, final_score, class_name, std::move(kps)});
    }

    auto iou = [](const cv::Rect2f &a, const cv::Rect2f &b)
    {
        float inter = (a & b).area();
        float uni = a.area() + b.area() - inter;
        return uni > 0 ? inter / uni : 0.0f;
    };

    std::vector<int> idx(dets.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int i, int j)
              { return dets[i].class_score > dets[j].class_score; });
    std::vector<char> suppressed(dets.size(), 0);
    std::vector<Detection> kept;
    kept.reserve(dets.size());
    for (size_t m = 0; m < idx.size(); ++m)
    {
        int i = idx[m];
        if (suppressed[i])
            continue;
        if (dets[i].class_score < conf_thr)
            continue;
        kept.push_back(std::move(dets[i]));
        for (size_t n = m + 1; n < idx.size(); ++n)
        {
            int j = idx[n];
            if (suppressed[j])
                continue;
            if (iou(kept.back().box, dets[j].box) > nms_thr)
                suppressed[j] = 1;
        }
    }

    std::cout << "Post Process produced " << kept.size() << " detections" << std::endl;
    std::cout << "\033[31m Post Process time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;

    // 8. 渲染
    begin_time = std::chrono::system_clock::now();
    for (const auto &det : kept)
    {
        cv::rectangle(img, det.box, cv::Scalar(255, 0, 0), LINE_SIZE);
        std::string text = det.class_name + ": " + std::to_string(static_cast<int>(det.disp_score * 100)) + "%";
        cv::Point origin(det.box.x, std::max(0.0f, det.box.y - 5));
        cv::putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, cv::Scalar(0, 0, 255), FONT_THICKNESS, cv::LINE_AA);
        for (const auto &kp : det.kpts)
        {
            cv::circle(img, kp, 2, cv::Scalar(0, 255, 0), -1);
        }
    }
    std::cout << "\033[31m Draw Result time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;

    cv::imwrite(IMG_SAVE_PATH, img);

    std::cout << "[INFO] Inference finished successfully." << std::endl;

    // 10. 释放任务
    hbDNNReleaseTask(task_handle);

    // 11. 释放内存
    hbSysFreeMem(&(input.sysMem[0]));
    for (int i = 0; i < output_count; ++i)
    {
        hbSysFreeMem(&(output[i].sysMem[0]));
    }
    delete[] output;

    // 12. 释放模型
    hbDNNRelease(packed_dnn_handle);

    return 0;
}
