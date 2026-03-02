# 模型输入处理说明（基于当前 YAML）

本说明基于以下 YAML 配置（节选）：

- `input_type_rt: bgr`
- `input_layout_rt: NHWC`
- `input_type_train: rgb`
- `input_layout_train: NCHW`
- `norm_type: data_scale`
- `scale_value: 0.0039215686`（= 1/255）
- `preprocess_on: True`

`preprocess_on: True` 表示 **模型内插入了预处理节点（HzPreprocess）**，会完成：
- `input_type_rt` → `input_type_train`（BGR → RGB）
- `input_layout_rt` → `input_layout_train`（NHWC → NCHW）
- `norm_type` 指定的归一化（这里是 `* 1/255`）

> 关键点：
> - **不要重复做归一化（/255）**，避免双重预处理。
> - **不要手动做 `-128`**（建议使用 `input_offset=128`）。

---

## 模型产物与输入处理对照表

| 模型产物 | 输入通道 | 输入布局 | 输入 dtype | 归一化（/255） | 是否手动 -128 | 备注 |
|---|---|---|---|---|---|---|
| `fp32_model.onnx`（训练导出） | RGB | NCHW | float32 | 需要 | 否 | 这是原始训练模型，不含预处理节点 |
| `*_original_float_model.onnx` | BGR | NCHW | uint8 | 不需要 | 否（用 `input_offset=128`） | 已插入 HzPreprocess |
| `*_optimized_float_model.onnx` | BGR | NCHW | uint8 | 不需要 | 否（用 `input_offset=128`） | 同上 |
| `*_calibrated_model.onnx` | BGR | NCHW | uint8 | 不需要 | 否（用 `input_offset=128`） | 同上 |
| `*_quantized_model.onnx` | BGR | NCHW | uint8 | 不需要 | 否（用 `input_offset=128`） | 同上 |
| `*.bin`（板端运行） | BGR | NHWC | uint8 | 不需要 | 否 | 运行时直接喂 RT 输入格式(存疑) |

---

## 使用（量化 onnx）

- 输入：BGR + NCHW + uint8
- 预处理：resize 到模型输入尺寸即可
- 推理：`HB_ONNXRuntime.run(..., input_offset=128)`

## 使用 (量化 bin)(存疑，测试无结果)

- 输入: BGR + NHWC + unit8
- 预处理: resize 到模型输入尺寸即可
- 推理: 参照 `BPU_API`

---

## 现方案描述

### 方案 A：RGB + NCHW 量化 bin（手动 -128）

- 模型输入：RGB + NCHW + int8（由 uint8 手动偏移到 int8）
- 预处理：resize 到模型输入尺寸；手动做 `-128`
- 量化/反量化：模型内部不含反量化节点
- 后处理：按常规检测后处理
- 备注：适合想明确控制输入偏移的场景
- 工具链跑评估结果：单线程latency: 6ms ![alt text](391af5f9b5a795d3d7ba3b35316b7df6.jpg)

### 方案 B：NV12 + NCHW 量化 bin（BGR2NV12）

- 模型输入：NV12 + NCHW + uint8
- 预处理：BGR 转 NV12；resize 到模型输入尺寸
- 量化/反量化：模型内部不含反量化节点
- 后处理：按常规检测后处理
- 备注：避免 RGB 通道转换在 CPU 上的额外开销时可考虑
- 工具链跑评估结果：单线程latency: 5.8ms![alt text](74a7e428ee7366c9d362357bd4c5f1d8.jpg)

### 方案 C：原始 onnx 量化后去反量化（手动 -128 + 后处理反量化）

- 模型输入：RGB + NCHW + int8（由 uint8 手动偏移到 int8）
- 预处理：resize 到模型输入尺寸；手动做 `-128`
- 量化/反量化：模型侧移除反量化节点；后处理自行反量化
- 后处理：先筛选再计算，分数不合格的 grid cell 不做反量化与计算
- 备注：通过减少CPU处理反量化来降时延
- 工具链跑评估结果：单线程latency: 4.9ms ![alt text](92e3e7128c5fe6728d9cf48319c2e32c.jpg)

### 方案 D：修改版 onnx 量化后去反量化（同方案 C 逻辑）

- 模型输入：与方案 A/B 对应的修改版输入格式
- 预处理：与对应输入格式一致
- 量化/反量化：模型侧移除反量化节点；后处理自行反量化
- 后处理：先筛选再计算，跳过低分 grid cell 的反量化与计算
- 备注：在修改版模型基础上进一步压缩后处理开销(单线程latency: 不去除反量化节点的时延 /2 )

> 未包含预处理及后处理时延，已知RGB手动-128转int8的处理在相机分辨率640x640下需要5ms，BGR2NV12还需要测试

>目前，方案B应为最稳妥的方案

## 参考文档
https://forum.d-robotics.cc/t/topic/28254 [RGB/INT8输入注意事项]

https://forum.d-robotics.cc/t/topic/28555
[RGB输入时quantized.onnx和bin的一致性对比方法]

https://forum.d-robotics.cc/t/topic/28554
[RGB模型的量化部署实战-以YOLOv5为例]

https://forum.d-robotics.cc/t/topic/28737
[[RDK X5][算法工具链]为什么PTQ rgb(nhwc)转换模式下的bin文件, 实际能得到正确结果的前处理却是nchw前处理方式??]

https://forum.d-robotics.cc/t/topic/25001
[反量化节点的融合实现]

https://blog.csdn.net/u013250861/article/details/139909317
[地平线-算法工具链：模型精度验证及调优建议]

https://developer.d-robotics.cc/api/v1/fileData/x5_doc-v126cn/runtime/source/bpu_sdk_api/bpu_sdk_api.html#id15[ BPU SDK API手册]

https://developer.d-robotics.cc/api/v1/fileData/x5_doc-v126cn/oe_mapper/source/faststart/quickstart.html#board-cpp-env[算法模型PTQ量化+上板 快速上手]