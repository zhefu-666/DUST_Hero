#include "threads/pipeline.h"
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <openrm/cudatools.h>

using namespace rm;
using namespace nvinfer1;
using namespace nvonnxparser;

void Pipeline::init_classifier() {
    auto param = Param::get_instance();

    // 检查是否启用分类器
    classifier_enabled_ = (*param)["Model"]["Classifier"]["Enable"];
    if (!classifier_enabled_) {
        rm::message("Classifier disabled", rm::MSG_WARNING);
        return;
    }

    // 获取配置参数
    std::string onnx_file   = (*param)["Model"]["Classifier"]["DirONNX"];
    std::string engine_file = (*param)["Model"]["Classifier"]["DirEngine"];
    classifier_infer_width_  = (*param)["Model"]["Classifier"]["InferWidth"];
    classifier_infer_height_ = (*param)["Model"]["Classifier"]["InferHeight"];
    classifier_class_num_    = (*param)["Model"]["Classifier"]["ClassNum"];

    // 初始化 CUDA 流
    if (!rm::initCudaStream(&classifier_stream_)) {
        rm::message("Failed to init classifier CUDA stream", rm::MSG_ERROR);
        classifier_enabled_ = false;
        return;
    }

    // 加载模型
    if (access(engine_file.c_str(), F_OK) == 0) {
        // 如果 engine 文件存在，直接加载
        if (!rm::initTrtEngine(engine_file, &classifier_context_)) {
            rm::message("Failed to load classifier engine", rm::MSG_ERROR);
            classifier_enabled_ = false;
            return;
        }
        rm::message("Classifier engine loaded: " + engine_file, rm::MSG_OK);
    } else if (access(onnx_file.c_str(), F_OK) == 0) {
        // 如果只有 onnx 文件，转换并加载
        if (!rm::initTrtOnnx(onnx_file, engine_file, &classifier_context_, 1U)) {
            rm::message("Failed to convert classifier ONNX", rm::MSG_ERROR);
            classifier_enabled_ = false;
            return;
        }
        rm::message("Classifier ONNX converted and loaded: " + onnx_file, rm::MSG_OK);
    } else {
        rm::message("No classifier model file found!", rm::MSG_ERROR);
        rm::message("Expected: " + onnx_file, rm::MSG_ERROR);
        classifier_enabled_ = false;
        return;
    }

    // 分配分类器缓冲区
    rm::mallocClassifyBuffer(
        &classifier_input_host_buffer_,
        &classifier_input_device_buffer_,
        &classifier_output_device_buffer_,
        &classifier_output_host_buffer_,
        classifier_infer_width_,
        classifier_infer_height_,
        classifier_class_num_
    );

    rm::message("Classifier initialized successfully", rm::MSG_OK);
    rm::message("  Model: tiny_resnet.onnx", rm::MSG_OK);
    rm::message("  Input size: " + std::to_string(classifier_infer_width_) + "x" + std::to_string(classifier_infer_height_), rm::MSG_OK);
    rm::message("  Classes: " + std::to_string(classifier_class_num_), rm::MSG_OK);
}

bool Pipeline::classifier(std::shared_ptr<rm::Frame> frame) {
    if (!classifier_enabled_ || classifier_context_ == nullptr) {
        return true;  // 如果分类器未启用，返回 true 继续流程
    }

    if (frame->yolo_list.empty()) {
        return true;  // 没有检测到装甲板，返回 true 继续流程
    }

    // 对每个检测到的装甲板进行分类
    for (auto& yolo_rect : frame->yolo_list) {
        // 使用 box 获取 ROI 区域，稍微扩展一下
        cv::Rect roi_rect = yolo_rect.box;
        
        int padding = 5;
        roi_rect.x = std::max(0, roi_rect.x - padding);
        roi_rect.y = std::max(0, roi_rect.y - padding);
        roi_rect.width = std::min(frame->width - roi_rect.x, roi_rect.width + 2 * padding);
        roi_rect.height = std::min(frame->height - roi_rect.y, roi_rect.height + 2 * padding);
        
        if (roi_rect.width <= 0 || roi_rect.height <= 0) {
            continue;
        }

        // 提取 ROI 并调整大小
        cv::Mat roi = (*frame->image)(roi_rect);
        cv::Mat resized_roi;
        cv::resize(roi, resized_roi, cv::Size(classifier_infer_width_, classifier_infer_height_));

        // 将图像数据复制到分类器缓冲区
        rm::memcpyClassifyBuffer(
            resized_roi.data,
            classifier_input_host_buffer_,
            classifier_input_device_buffer_,
            classifier_infer_width_,
            classifier_infer_height_
        );

        // 执行推理
        rm::detectEnqueue(
            classifier_input_device_buffer_,
            classifier_output_device_buffer_,
            &classifier_context_,
            &classifier_stream_
        );

        // 获取输出
        rm::detectOutputClassify(
            classifier_output_host_buffer_,
            classifier_output_device_buffer_,
            &classifier_stream_,
            classifier_class_num_
        );

        // 找到最大概率的类别
        int max_class = 0;
        float max_prob = classifier_output_host_buffer_[0];
        for (int i = 1; i < classifier_class_num_; i++) {
            if (classifier_output_host_buffer_[i] > max_prob) {
                max_prob = classifier_output_host_buffer_[i];
                max_class = i;
            }
        }

        // 更新 yolo_rect 的类别 ID（使用分类器结果）
        yolo_rect.class_id = max_class;
        
        // 打印分类结果（调试用）
        if (Data::pipeline_delay_flag) {
            rm::message("Classifier result: class=" + std::to_string(max_class) + 
                       ", prob=" + std::to_string(max_prob), rm::MSG_OK);
        }
    }

    return true;
}
