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

    std::cout << "[CLASSIFIER] 初始化数字分类器..." << std::endl;

    // 检查是否启用分类器
    classifier_enabled_ = (*param)["Model"]["Classifier"]["Enable"];
    if (!classifier_enabled_) {
        std::cout << "[CLASSIFIER] 分类器已禁用" << std::endl;
        return;
    }

    // 获取配置参数
    std::string onnx_file   = (*param)["Model"]["Classifier"]["DirONNX"];
    std::string engine_file = (*param)["Model"]["Classifier"]["DirEngine"];
    classifier_infer_width_  = (*param)["Model"]["Classifier"]["InferWidth"];
    classifier_infer_height_ = (*param)["Model"]["Classifier"]["InferHeight"];
    classifier_class_num_    = (*param)["Model"]["Classifier"]["ClassNum"];

    std::cout << "[CLASSIFIER] ONNX: " << onnx_file << std::endl;
    std::cout << "[CLASSIFIER] Engine: " << engine_file << std::endl;
    std::cout << "[CLASSIFIER] 输入尺寸: " << classifier_infer_width_ << "x" << classifier_infer_height_ << std::endl;
    std::cout << "[CLASSIFIER] 类别数: " << classifier_class_num_ << std::endl;

    // 初始化 CUDA 流
    if (!rm::initCudaStream(&classifier_stream_)) {
        std::cout << "[CLASSIFIER] 错误: 无法初始化 CUDA 流" << std::endl;
        classifier_enabled_ = false;
        return;
    }
    std::cout << "[CLASSIFIER] CUDA 流初始化成功" << std::endl;

    // 加载模型
    if (access(engine_file.c_str(), F_OK) == 0) {
        std::cout << "[CLASSIFIER] 加载 Engine 文件..." << std::endl;
        if (!rm::initTrtEngine(engine_file, &classifier_context_)) {
            std::cout << "[CLASSIFIER] 错误: 无法加载 Engine 文件" << std::endl;
            classifier_enabled_ = false;
            return;
        }
        std::cout << "[CLASSIFIER] Engine 加载成功" << std::endl;
    } else if (access(onnx_file.c_str(), F_OK) == 0) {
        std::cout << "[CLASSIFIER] 转换 ONNX 为 Engine..." << std::endl;
        if (!rm::initTrtOnnx(onnx_file, engine_file, &classifier_context_, 1U)) {
            std::cout << "[CLASSIFIER] 错误: ONNX 转换失败" << std::endl;
            classifier_enabled_ = false;
            return;
        }
        std::cout << "[CLASSIFIER] ONNX 转换并加载成功" << std::endl;
    } else {
        std::cout << "[CLASSIFIER] 错误: 找不到模型文件!" << std::endl;
        std::cout << "[CLASSIFIER] 期望: " << onnx_file << std::endl;
        classifier_enabled_ = false;
        return;
    }

    // 分配分类器缓冲区
    std::cout << "[CLASSIFIER] 分配缓冲区..." << std::endl;
    rm::mallocClassifyBuffer(
        &classifier_input_host_buffer_,
        &classifier_input_device_buffer_,
        &classifier_output_device_buffer_,
        &classifier_output_host_buffer_,
        classifier_infer_width_,
        classifier_infer_height_,
        classifier_class_num_
    );

    std::cout << "[CLASSIFIER] ✓ 数字分类器初始化完成!" << std::endl;
    std::cout << "[CLASSIFIER]   模型: number_classifier.onnx" << std::endl;
    std::cout << "[CLASSIFIER]   输入: " << classifier_infer_width_ << "x" << classifier_infer_height_ << std::endl;
    std::cout << "[CLASSIFIER]   类别: " << classifier_class_num_ << " (数字 0-9)" << std::endl;
}

bool Pipeline::classifier(std::shared_ptr<rm::Frame> frame) {
    if (frame->yolo_list.empty()) {
        return true;  // 没有检测到装甲板，返回 true 继续流程
    }

    // ============ 无论分类器是否启用，都需要设置 color_id ============
    // FP 模型: class_id 0-6 是蓝色, 7-12 是红色
    for (auto& yolo_rect : frame->yolo_list) {
        int original_class_id = yolo_rect.class_id;
        if (original_class_id >= 0 && original_class_id <= 6) {
            yolo_rect.color_id = 0;  // 蓝色
        } else if (original_class_id >= 7 && original_class_id <= 12) {
            yolo_rect.color_id = 1;  // 红色
        }
    }

    // 如果分类器未启用，直接返回（color_id 已设置）
    if (!classifier_enabled_ || classifier_context_ == nullptr) {
        return true;
    }

    // 对每个检测到的装甲板进行数字分类
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
            std::string color_str = (yolo_rect.color_id == 0) ? "蓝" : "红";
            std::cout << "[CLASSIFIER] " << color_str << " 装甲板分类结果: " 
                      << max_class << " (置信度: " << max_prob << ")" << std::endl;
        }
    }

    return true;
}
