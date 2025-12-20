#include "threads/pipeline.h"
#include "data_manager/base.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <atomic>
#include <map>

// 模型 0526.engine 的类别名称映射 (ClassNum=13)
// 根据测试: 蓝3=0, 红3=7，推断类别顺序
static const std::map<int, std::string> YOLO_CLASS_NAMES = {
    {0, "B3"},   // 蓝色3号步兵
    {1, "B4"},   // 蓝色4号步兵
    {2, "B5"},   // 蓝色5号步兵
    {3, "B-S"},  // 蓝色哨兵
    {4, "B1"},   // 蓝色英雄
    {5, "B2"},   // 蓝色工程
    {6, "B-T"},  // 蓝色前哨/基地
    {7, "R3"},   // 红色3号步兵
    {8, "R4"},   // 红色4号步兵
    {9, "R5"},   // 红色5号步兵
    {10, "R-S"}, // 红色哨兵
    {11, "R1"},  // 红色英雄
    {12, "R2"},  // 红色工程
};

// 分类器数字映射（启用分类器后使用）
// 数字分类器输出 class_id 0-9 对应数字 0-9
// 但是装甲板实际数字只有 0-6 (哨兵/基地=0, 英雄=1, 工程=2, 步兵3/4/5=3/4/5, 前哨=6)
// 如果分类器输出 7/8/9，可能是识别错误
static std::string get_classifier_number_name(int class_id) {
    // 分类器输出的数字直接显示
    if (class_id >= 0 && class_id <= 9) {
        return std::to_string(class_id);
    }
    return "?" + std::to_string(class_id);
}

static std::string get_yolo_class_name(int class_id) {
    auto it = YOLO_CLASS_NAMES.find(class_id);
    if (it != YOLO_CLASS_NAMES.end()) {
        return it->second;
    }
    return "?" + std::to_string(class_id);
}

// 外部声明
extern std::mutex g_frame_mutex;
extern cv::Mat g_display_frame;
extern bool g_new_frame_available;
extern std::atomic<bool> g_running;

// 全局检测结果
std::mutex g_detection_mutex;
std::vector<rm::YoloRect> g_detections;
bool g_detection_updated = false;
bool g_classifier_enabled = false;  // 标记分类器是否启用

void update_global_detections(const std::vector<rm::YoloRect>& detections) {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    g_detections = detections;
    g_detection_updated = true;
}

void set_classifier_enabled(bool enabled) {
    g_classifier_enabled = enabled;
}

static bool check_model_exists() {
    std::vector<std::string> model_paths = {
        "/home/hero/TJURM-2024/data/uniconfig/models/0526.engine",
        "/home/hero/DUST_Hero/data/uniconfig/models/0526.engine"
    };
    for (const auto& path : model_paths) {
        std::ifstream f(path);
        if (f.good()) return true;
    }
    return false;
}

void Pipeline::display_thread() {
    std::cout << "\n========================================\n";
    std::cout << "Camera Stream Display Thread Started\n";
    std::cout << "========================================\n";
    
    bool model_exists = check_model_exists();
    std::cout << "Mode: Detection Display with Bounding Boxes\n";
    if (model_exists) std::cout << "Note: Model file found, detection active\n";
    std::cout << "Press 'q' or Ctrl+C to exit\n";
    std::cout << "========================================\n\n";
    
    // 检查分类器配置
    auto param = Param::get_instance();
    bool classifier_cfg_enabled = (*param)["Model"]["Classifier"]["Enable"];
    g_classifier_enabled = classifier_cfg_enabled;
    if (g_classifier_enabled) {
        std::cout << "[Display] Number classifier enabled - showing digit labels\n";
    } else {
        std::cout << "[Display] Classifier disabled - showing YOLO class labels\n";
    }
    
    bool window_available = false;
    if (Data::imshow_flag) {
        try {
            cv::namedWindow("Armor Detection", cv::WINDOW_NORMAL);
            cv::resizeWindow("Armor Detection", 960, 720);
            window_available = true;
            std::cout << "[Display] Window created\n";
        } catch (const cv::Exception& e) {
            std::cout << "[Display] Window failed: " << e.what() << "\n";
        }
    }
    
    cv::Mat local_frame;
    std::vector<rm::YoloRect> local_detections;
    unsigned long frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 主循环 - 检查g_running
    while (g_running) {
        bool have_new_frame = false;
        
        // 获取帧
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            if (g_new_frame_available && !g_display_frame.empty()) {
                g_display_frame.copyTo(local_frame);
                g_new_frame_available = false;
                have_new_frame = true;
                frame_count++;
            }
        }
        
        // 获取检测结果
        {
            std::lock_guard<std::mutex> lock(g_detection_mutex);
            if (g_detection_updated) {
                local_detections = g_detections;
                g_detection_updated = false;
            }
        }
        
        if (have_new_frame && !local_frame.empty()) {
            // 绘制检测框
            for (const auto& det : local_detections) {
                // 根据 color_id 判断颜色
                // color_id: 0=蓝色, 1=红色
                cv::Scalar box_color;
                std::string color_prefix;
                if (det.color_id == 0) {
                    box_color = cv::Scalar(255, 0, 0);  // BGR: 蓝色
                    color_prefix = "B";
                } else {
                    box_color = cv::Scalar(0, 0, 255);  // BGR: 红色
                    color_prefix = "R";
                }
                
                // 矩形框
                cv::rectangle(local_frame, det.box, box_color, 2);
                
                // 四点框
                if (det.four_points.size() == 4) {
                    for (int i = 0; i < 4; i++) {
                        cv::line(local_frame, det.four_points[i], det.four_points[(i+1)%4], 
                                cv::Scalar(0,255,255), 2);
                        cv::circle(local_frame, det.four_points[i], 4, cv::Scalar(0,255,0), -1);
                    }
                }
                
                // 标签: 根据是否启用分类器选择显示内容
                std::string label;
                if (g_classifier_enabled) {
                    // 分类器启用: 显示颜色前缀 + 分类器识别的数字
                    // det.color_id 已经在 classifier() 中根据原始 YOLO class_id 设置
                    // det.class_id 是分类器输出的数字 (0-9)
                    label = color_prefix + get_classifier_number_name(det.class_id) + 
                           " " + std::to_string(int(det.confidence * 100)) + "%";
                } else {
                    // 分类器未启用: 显示 YOLO 原始类别名称
                    label = get_yolo_class_name(det.class_id) + 
                           " " + std::to_string(int(det.confidence * 100)) + "%";
                }
                cv::putText(local_frame, label, cv::Point(det.box.x, det.box.y - 5), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2);
            }
            
            // 信息显示
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            double fps = elapsed > 0 ? frame_count / elapsed : 0;
            
            cv::putText(local_frame, "Detections: " + std::to_string(local_detections.size()), 
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
            cv::putText(local_frame, "FPS: " + std::to_string(int(fps)), 
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
            
            // 显示分类器状态
            std::string classifier_status = g_classifier_enabled ? "Classifier: ON" : "Classifier: OFF";
            cv::putText(local_frame, classifier_status, 
                       cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                       g_classifier_enabled ? cv::Scalar(0,255,0) : cv::Scalar(0,165,255), 2);
            
            // 显示窗口
            if (window_available) {
                try {
                    cv::imshow("Armor Detection", local_frame);
                    int key = cv::waitKey(1);
                    if (key == 'q' || key == 'Q' || key == 27) {
                        std::cout << "\n[Display] Exit requested\n";
                        g_running = false;
                        break;
                    }
                } catch (const cv::Exception& e) {
                    // 忽略显示错误
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (window_available) {
        try {
            cv::destroyAllWindows();
        } catch (...) {}
    }
    
    std::cout << "[Display] Thread exiting, processed " << frame_count << " frames\n";
}
