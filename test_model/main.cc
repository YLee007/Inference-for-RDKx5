#define MODEL_PATH "/home/sunrise/Documents/Inference-for-RDKx5/autoaim/model/yolov5_detect_640x640_bayese_rgb.bin"

#define TESR_IMG_PATH "/home/sunrise/dataset/"

// 前处理方式选择, 0:Resize, 1:LetterBox
// Preprocessing method selection, 0: Resize, 1: LetterBox
#define RESIZE_TYPE 0
#define LETTERBOX_TYPE 1
#define PREPROCESS_TYPE LETTERBOX_TYPE

// 推理结果保存路径
// Path where the inference result will be saved
#define IMG_SAVE_PATH "./output/"

// 模型的类别数量, 默认80
// Number of classes in the model, default is 80
#define CLASSES_NUM 9

// NMS的阈值, 默认0.45
// Non-Maximum Suppression (NMS) threshold, default is 0.45
#define NMS_THRESHOLD 0.7

// 分数阈值, 默认0.25
// Score threshold, default is 0.25
#define SCORE_THRESHOLD 0.30

#define LINE_SIZE 2
#define FONT_SIZE 0.6
#define FONT_THICKNESS 2

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

//Names
std::vector<std::string> object_names = {
    "G", "1", "2", "3", "4", "5", "O", "Bs", "Bb"}; 

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


    // 3. 利用OpenCV准备输入数据
    std::vector<std::string> img_paths;
    std::string tesr_path = TESR_IMG_PATH;
    std::vector<cv::String> glob_paths;
    cv::glob(tesr_path, glob_paths, false);
    if (glob_paths.empty())
    {
        cv::glob(tesr_path + "/*.*", glob_paths, false);
    }
    for (const auto &p : glob_paths)
    {
        std::string ext;
        auto dot = p.find_last_of('.');
        if (dot != std::string::npos)
        {
            ext = p.substr(dot);
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        }
        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" || ext == ".webp")
        {
            img_paths.push_back(p);
        }
    }

    if (img_paths.empty())
    {
        std::cerr << "No valid images found in " << TESR_IMG_PATH << std::endl;
        return -1;
    }

    hbDNNTensor input;
    input.properties = input_properties;
    hbSysAllocCachedMem(&input.sysMem[0], int(3 * input_H * input_W));

    hbDNNTensor *output = new hbDNNTensor[output_count];
    for (int i = 0; i < output_count; i++)
    {
        hbDNNTensorProperties &output_properties = output[i].properties;
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
        int out_aligned_size = output_properties.alignedByteSize;
        hbSysMem &mem = output[i].sysMem[0];
        hbSysAllocCachedMem(&mem, out_aligned_size);
    }

    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

    for (const auto &img_file : img_paths)
    {
        float y_scale = 1.0;
        float x_scale = 1.0;
        int x_shift = 0;
        int y_shift = 0;

        cv::Mat img = cv::imread(img_file);
        if (img.empty())
        {
            std::cerr << "Failed to read image: " << img_file << std::endl;
            continue;
        }
        std::cout << "img path: " << img_file << std::endl;
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

        hbDNNTaskHandle_t task_handle = nullptr;
        hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param);

        hbDNNWaitTaskDone(task_handle, 0);
        std::cout << "\033[31m forward time = " << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - begin_time).count() / 1000.0 << " ms\033[0m" << std::endl;

        // 7. YOLO 后处理（参考 OpenvinoInfer.cpp）
        begin_time = std::chrono::system_clock::now();

        const float conf_thr = SCORE_THRESHOLD;
        const float nms_thr = NMS_THRESHOLD;
        const int detect_color = -1; // -1 保留红/蓝，丢弃灰/紫

        hbDNNTensor &pp_output = output[0];
        hbSysFlushMem(&(pp_output.sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

        auto &vshape = pp_output.properties.validShape;
        auto qtype = pp_output.properties.quantiType;
        int shift0 = 0;
        float scale0 = 1.0f;
        if (qtype == SHIFT && pp_output.properties.shift.shiftData)
            shift0 = pp_output.properties.shift.shiftData[0];
        if (qtype == SCALE && pp_output.properties.scale.scaleData)
            scale0 = pp_output.properties.scale.scaleData[0];
        std::cout << "[DEBUG] output quantiType=" << qtype
                  << " shift[0]=" << shift0
                  << " scale[0]=" << scale0 << std::endl;
        // Print output tensor shape info
        std::cout << "[DEBUG] output validShape numDim=" << vshape.numDimensions;
        for (int i = 0; i < vshape.numDimensions; ++i)
            std::cout << " dim" << i << "=" << vshape.dimensionSize[i];
        std::cout << std::endl;

        int nd = vshape.numDimensions;
        int rows = 0;
        int cols = 0;
        // Infer rows/cols based on common layouts
        if (nd == 4)
        {
            // Common YOLO export: (1, N, 22, 1)
            rows = static_cast<int>(vshape.dimensionSize[1]);
            cols = static_cast<int>(vshape.dimensionSize[2]);
        }
        else if (nd == 3)
        {
            rows = static_cast<int>(vshape.dimensionSize[1]);
            cols = static_cast<int>(vshape.dimensionSize[2]);
        }
        else if (nd == 2)
        {
            rows = static_cast<int>(vshape.dimensionSize[0]);
            cols = static_cast<int>(vshape.dimensionSize[1]);
        }
        else
        {
            std::cout << "Unsupported tensor dims: " << nd << std::endl;
            hbDNNReleaseTask(task_handle);
            continue;
        }

        // Fallback reshape if cols still incorrect
        if (cols != 22)
        {
            int64_t total = 1;
            for (int i = 0; i < nd; ++i)
                total *= vshape.dimensionSize[i];
            if (total % 22 == 0)
            {
                cols = 22;
                rows = static_cast<int>(total / 22);
            }
        }
        if (cols != 22)
        {
            std::cout << "Expect 22 columns, got " << cols << " (nd=" << nd << ")" << std::endl;
            hbDNNReleaseTask(task_handle);
            continue;
        }

        // Dequantize output if needed; otherwise use raw float
        float *data = nullptr;
        int64_t total_elems = static_cast<int64_t>(rows) * static_cast<int64_t>(cols);
        std::vector<float> out;
        if (qtype == SHIFT)
        {
            out.resize(total_elems);
            const int8_t *src = reinterpret_cast<int8_t *>(pp_output.sysMem[0].virAddr);
            int shift = (pp_output.properties.shift.shiftData ? pp_output.properties.shift.shiftData[0] : 0);
            float scale = 1.0f / static_cast<float>(1 << shift);
            for (int64_t i = 0; i < total_elems; ++i)
                out[i] = src[i] * scale;
            data = out.data();
        }
        else if (qtype == SCALE)
        {
            out.resize(total_elems);
            const int8_t *src = reinterpret_cast<int8_t *>(pp_output.sysMem[0].virAddr);
            float scale = (pp_output.properties.scale.scaleData ? pp_output.properties.scale.scaleData[0] : 1.0f);
            for (int64_t i = 0; i < total_elems; ++i)
                out[i] = src[i] * scale;
            data = out.data();
        }
        else
        {
            data = reinterpret_cast<float *>(pp_output.sysMem[0].virAddr);
        }
        auto sigmoid = [](float x)
        { return 1.0f / (1.0f + std::exp(-x)); };

        // Dump first few raw values to inspect layout
        int dump_vals = std::min<int64_t>(total_elems, 32);
        std::cout << "[DEBUG] first " << dump_vals << " values:";
        for (int i = 0; i < dump_vals; ++i)
            std::cout << " " << data[i];
        std::cout << std::endl;

        // Debug: dump first few rows to inspect coordinate scale
        int dump_rows = std::min(rows, 3);
        for (int di = 0; di < dump_rows; ++di)
        {
            const float *r = data + di * cols;
            std::cout << "[DEBUG] row" << di << " obj=" << sigmoid(r[8])
                      << " coords: " << r[0] << "," << r[1] << "," << r[2] << "," << r[3]
                      << " cls_max=" << *std::max_element(r + 13, r + 22)
                      << std::endl;
        }

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
        std::vector<cv::Rect> nms_boxes;
        std::vector<float> nms_scores;

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
                continue;
            if (detect_color == 0 && color_idx == 1)
                continue;
            if (detect_color == 1 && color_idx == 0)
                continue;

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

            // Use raw model outputs as pixel coords on resized input
            float x0 = r[0];
            float y0 = r[1];
            float x1 = r[2];
            float y1 = r[3];
            float x2 = r[4];
            float y2 = r[5];
            float x3 = r[6];
            float y3 = r[7];

            float tlx = scale_x(x0);
            float tly = scale_y(y0);
            float blx = scale_x(x1);
            float bly = scale_y(y1);
            float brx = scale_x(x2);
            float bry = scale_y(y2);
            float trx = scale_x(x3);
            float try_ = scale_y(y3);

            std::vector<cv::Point2f> kps{{blx, bly}, {tlx, tly}, {trx, try_}, {brx, bry}};

            float minx = std::min(std::min(tlx, blx), std::min(brx, trx));
            float maxx = std::max(std::max(tlx, blx), std::max(brx, trx));
            float miny = std::min(std::min(tly, bly), std::min(bry, try_));
            float maxy = std::max(std::max(tly, bly), std::max(bry, try_));
            cv::Rect rect(minx, miny, std::max(0.0f, maxx - minx), std::max(0.0f, maxy - miny));
            if (rect.width <= 0 || rect.height <= 0)
                continue;

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
            nms_boxes.push_back(rect);
            nms_scores.push_back(final_score);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(nms_boxes, nms_scores, conf_thr, nms_thr, indices);
        std::vector<Detection> kept;
        kept.reserve(indices.size());
        for (int idx : indices)
        {
            if (idx >= 0 && idx < static_cast<int>(dets.size()))
            {
                kept.push_back(std::move(dets[idx]));
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

        std::string out_path = IMG_SAVE_PATH;
        if (img_paths.size() > 1)
        {
            std::string base_name = img_file;
            auto pos = base_name.find_last_of("/\\");
            std::string dir = (pos == std::string::npos) ? "" : base_name.substr(0, pos + 1);
            std::string file = (pos == std::string::npos) ? base_name : base_name.substr(pos + 1);
            auto dot = file.find_last_of('.');
            std::string stem = (dot == std::string::npos) ? file : file.substr(0, dot);
            out_path = dir + stem + "_cpp_result.jpg";
        }
        cv::imwrite(out_path, img);
        std::cout << "[INFO] Saved result to " << out_path << std::endl;

        hbDNNReleaseTask(task_handle);
    }

    std::cout << "[INFO] Inference finished successfully." << std::endl;

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
