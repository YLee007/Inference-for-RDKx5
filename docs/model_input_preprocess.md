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
| `*.bin`（板端运行） | BGR | NHWC | uint8 | 不需要 | 否 | 运行时直接喂 RT 输入格式 |

---

## 使用（量化 onnx）

- 输入：BGR + NCHW + uint8
- 预处理：resize 到模型输入尺寸即可
- 推理：`HB_ONNXRuntime.run(..., input_offset=128)`

## 使用 (量化 bin)

- 输入: BGR + NHWC + unit8
- 预处理: resize 到模型输入尺寸即可
- 推理: 参照 `BPU_API`
