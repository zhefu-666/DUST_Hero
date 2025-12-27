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
#include <openrm/pointer/pointer.h>

// 模型 0526.engine 的类别名称映射 (ClassNum=13)
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

static std::string get_classifier_number_name(int class_id) {
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
bool g_classifier_enabled = false;

// 全局装甲板结果（用于重投影）
std::mutex g_armor_mutex;
std::vector<rm::Armor> g_armors;
bool g_armor_updated = false;

void update_global_detections(const std::vector<rm::YoloRect>& detections) {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    g_detections = detections;
    g_detection_updated = true;
}

void update_global_armors(const std::vector<rm::Armor>& armors) {
    std::lock_guard<std::mutex> lock(g_armor_mutex);
    g_armors = armors;
    g_armor_updated = true;
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
    
    // 获取重投影参数
    float big_width    = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    float big_height   = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    float small_width  = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    float small_height = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];
    
    // 获取重投影贴图路径
    std::string small_path = (*param)["Debug"]["SmallDecal"];
    std::string big_path   = (*param)["Debug"]["BigDecal"];
    
    // 初始化重投影（加载贴图）
    if (Data::reprojection_flag) {
        rm::initReprojection(small_width, small_height, big_width, big_height, small_path, big_path);
        std::cout << "[Display] Reprojection initialized with images" << std::endl;
    }
    
    bool window_available = false;
    bool reproj_window_available = false;
    
    if (Data::imshow_flag) {
        try {
            cv::namedWindow("Armor Detection", cv::WINDOW_NORMAL);
            cv::resizeWindow("Armor Detection", 960, 720);
            window_available = true;
            std::cout << "[Display] Main window created\n";
        } catch (const cv::Exception& e) {
            std::cout << "[Display] Main window failed: " << e.what() << "\n";
        }
        
        // 创建重投影窗口
        if (Data::reprojection_flag) {
            try {
                cv::namedWindow("Reprojection", cv::WINDOW_NORMAL);
                cv::resizeWindow("Reprojection", 640, 480);
                reproj_window_available = true;
                std::cout << "[Display] Reprojection window created\n";
            } catch (const cv::Exception& e) {
                std::cout << "[Display] Reprojection window failed: " << e.what() << "\n";
            }
        }
    }
    
    cv::Mat local_frame;
    cv::Mat reproj_frame;  // 重投影显示帧
    std::vector<rm::YoloRect> local_detections;
    std::vector<rm::Armor> local_armors;
    unsigned long frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 创建空白重投影画面
    reproj_frame = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::putText(reproj_frame, "Waiting for armor detection...", 
               cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(128,128,128), 2);
    
    // 主循环
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
        
        // 获取装甲板结果（用于重投影）
        {
            std::lock_guard<std::mutex> lock(g_armor_mutex);
            if (g_armor_updated) {
                local_armors = g_armors;
                g_armor_updated = false;
            }
        }
        
        if (have_new_frame && !local_frame.empty()) {
            // 绘制检测框
            for (const auto& det : local_detections) {
                cv::Scalar box_color;
                std::string color_prefix;
                if (det.color_id == 0) {
                    box_color = cv::Scalar(255, 0, 0);  // BGR: 蓝色
                    color_prefix = "B";
                } else {
                    box_color = cv::Scalar(0, 0, 255);  // BGR: 红色
                    color_prefix = "R";
                }
                
                cv::rectangle(local_frame, det.box, box_color, 2);
                
                if (det.four_points.size() == 4) {
                    for (int i = 0; i < 4; i++) {
                        cv::line(local_frame, det.four_points[i], det.four_points[(i+1)%4], 
                                cv::Scalar(0,255,255), 2);
                        cv::circle(local_frame, det.four_points[i], 4, cv::Scalar(0,255,0), -1);
                    }
                }
                
                std::string label;
                if (g_classifier_enabled) {
                    label = color_prefix + get_classifier_number_name(det.class_id) + 
                           " " + std::to_string(int(det.confidence * 100)) + "%";
                } else {
                    label = get_yolo_class_name(det.class_id) + 
                           " " + std::to_string(int(det.confidence * 100)) + "%";
                }
                cv::putText(local_frame, label, cv::Point(det.box.x, det.box.y - 5), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2);
            }
            
            // 更新重投影显示（直接使用YOLO检测的四点）
            if (Data::reprojection_flag && reproj_window_available) {
                if (!local_detections.empty()) {
                    // 有检测，进行重投影
                    reproj_frame = local_frame.clone();
                    
                    for (const auto& det : local_detections) {
                        if (det.four_points.size() == 4) {
                            // 根据class_id判断大小装甲板 (1号英雄和前哨站/基地是大装甲板)
                            // YOLO类别: 0-6蓝方, 7-12红方
                            // 类别映射: 0,7->1号(英雄大), 5,12->前哨(大), 6,13->基地(大)
                            int class_mod = det.class_id % 7;  // 0-6 或 7-13 -> 0-6
                            rm::ArmorSize size = rm::ARMOR_SIZE_SMALL_ARMOR;
                            if (class_mod == 0 || class_mod == 5 || class_mod == 6) {
                                size = rm::ARMOR_SIZE_BIG_ARMOR;
                            }
                            
                            // 设置重投影参数并绘制
                            rm::paramReprojection(small_width, small_height, big_width, big_height);
                            rm::setReprojection(local_frame, reproj_frame, det.four_points, size);
                        }
                    }
                    
                    // 添加状态信息
                    cv::putText(reproj_frame, "Detections: " + std::to_string(local_detections.size()), 
                               cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
                } else {
                    // 无检测，显示等待信息
                    reproj_frame = local_frame.clone();
                    cv::putText(reproj_frame, "No armor detected", 
                               cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,165,255), 2);
                }
            }
            
            // 信息显示
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            double fps = elapsed > 0 ? frame_count / elapsed : 0;
            
            cv::putText(local_frame, "Detections: " + std::to_string(local_detections.size()), 
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
            cv::putText(local_frame, "FPS: " + std::to_string(int(fps)), 
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
            
            std::string classifier_status = g_classifier_enabled ? "Classifier: ON" : "Classifier: OFF";
            cv::putText(local_frame, classifier_status, 
                       cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                       g_classifier_enabled ? cv::Scalar(0,255,0) : cv::Scalar(0,165,255), 2);
            
            // 显示主窗口
            if (window_available) {
                try {
                    cv::imshow("Armor Detection", local_frame);
                } catch (const cv::Exception& e) {
                    // 忽略显示错误
                }
            }
            
            // 显示重投影窗口
            if (reproj_window_available && !reproj_frame.empty()) {
                try {
                    cv::imshow("Reprojection", reproj_frame);
                } catch (const cv::Exception& e) {
                    // 忽略显示错误
                }
            }
            
            // 处理按键
            if (window_available || reproj_window_available) {
                int key = cv::waitKey(1);
                if (key == 'q' || key == 'Q' || key == 27) {
                    std::cout << "\n[Display] Exit requested\n";
                    g_running = false;
                    break;
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (window_available || reproj_window_available) {
        try {
            cv::destroyAllWindows();
        } catch (...) {}
    }
    
    std::cout << "[Display] Thread exiting, processed " << frame_count << " frames\n";
}
