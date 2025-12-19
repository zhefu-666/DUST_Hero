#include "threads/pipeline.h"
#include "data_manager/base.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <fstream>

// 外部声明（来自init.cpp）
extern std::mutex g_frame_mutex;
extern cv::Mat g_display_frame;
extern bool g_new_frame_available;

// 检查模型文件是否存在
static bool check_model_exists() {
    // 检查常见的模型文件路径
    std::vector<std::string> model_paths = {
        "/home/hero/TJURM-2024/data/uniconfig/models/armor.engine",
        "/home/hero/DUST_Hero/data/uniconfig/models/armor.engine",
        "data/uniconfig/models/armor.engine"
    };
    
    for (const auto& path : model_paths) {
        std::ifstream f(path);
        if (f.good()) {
            return true;
        }
    }
    return false;
}

void Pipeline::display_thread() {
    std::cout << "\n========================================\n";
    std::cout << "Camera Stream Display Thread Started\n";
    std::cout << "========================================\n";
    
    // 检查是否有模型文件
    // 如果没有模型文件，image_thread不会工作，我们可以安全地显示窗口
    bool model_exists = check_model_exists();
    bool should_show_window = Data::imshow_flag && !model_exists;
    
    if (Data::imshow_flag) {
        if (model_exists) {
            std::cout << "Mode: Frame Monitor (GUI handled by image_thread)\n";
            std::cout << "Note: Model file found, detection active\n";
        } else {
            std::cout << "Mode: Display Window (showing raw camera frames)\n";
            std::cout << "Note: No model file, this thread handles display\n";
        }
    } else {
        std::cout << "Mode: Frame Processing Only (no -s flag)\n";
    }
    
    std::cout << "Resolution: 1440x1080\n";
    std::cout << "========================================\n\n";
    
    // 只在没有模型文件且启用imshow时创建窗口
    bool window_available = false;
    if (should_show_window) {
        try {
            cv::namedWindow("Camera Stream - Press q to exit", cv::WINDOW_AUTOSIZE);
            window_available = true;
            std::cout << "[Display] ✓ Window created successfully\n";
        } catch (const cv::Exception& e) {
            std::cout << "[Display] ⚠ Window creation failed: " << e.what() << "\n";
            window_available = false;
        } catch (...) {
            std::cout << "[Display] ⚠ Unexpected error creating window\n";
            window_available = false;
        }
    }
    
    int display_fps = 25;
    int frame_delay = 1000 / display_fps;
    
    unsigned long frame_count = 0;
    bool debug_once = true;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 用于存储帧副本的本地变量（在锁外使用）
    cv::Mat local_frame;
    bool have_new_frame = false;
    
    while (true) {
        have_new_frame = false;
        
        // 在锁内快速复制帧数据
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            
            if (g_new_frame_available && !g_display_frame.empty()) {
                // 深拷贝帧数据到本地变量
                g_display_frame.copyTo(local_frame);
                g_new_frame_available = false;
                have_new_frame = true;
                frame_count++;
            }
        }
        // 锁在此处释放
        
        // 在锁外处理帧和GUI操作
        if (have_new_frame && !local_frame.empty()) {
            if (debug_once) {
                std::cout << "[Display] ✓ First frame received!\n";
                std::cout << "  Size: " << local_frame.cols << "x" << local_frame.rows << "\n";
                std::cout << "  Channels: " << local_frame.channels() << "\n";
                debug_once = false;
            }
            
            // 显示窗口（如果可用且启用）- 在锁外进行
            if (window_available) {
                try {
                    cv::imshow("Camera Stream - Press q to exit", local_frame);
                    int key = cv::waitKey(1);
                    if (key == 'q' || key == 27 || key == 'Q') {  // q, ESC, or Q
                        std::cout << "[Display] User pressed exit key\n";
                        break;
                    }
                } catch (...) {
                    // 窗口被关闭或其他错误，继续处理帧
                }
            }
            
            // 计算实际FPS
            if (frame_count % 200 == 0) {
                auto current_time = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(current_time - start_time).count();
                double actual_fps = elapsed > 0 ? frame_count / elapsed : 0;
                std::cout << "[Display] Frames: " << frame_count 
                          << " | FPS: " << std::fixed << std::setprecision(1) << actual_fps << "\n";
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(frame_delay));
    }
    
    // 清理窗口
    if (window_available) {
        try {
            cv::destroyAllWindows();
        } catch (...) {
        }
    }
}
