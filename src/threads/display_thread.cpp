#include "threads/pipeline.h"
#include "data_manager/base.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <iomanip>

// 外部声明（来自init.cpp）
extern std::mutex g_frame_mutex;
extern cv::Mat g_display_frame;
extern bool g_new_frame_available;

void Pipeline::display_thread() {
    std::cout << "\n========================================\n";
    std::cout << "Camera Stream Display Thread Started\n";
    std::cout << "========================================\n";
    
    if (Data::imshow_flag) {
        std::cout << "Mode: Display Window (imshow enabled with -s)\n";
    } else {
        std::cout << "Mode: Frame Processing Only\n";
        std::cout << "Tip: Use -s flag to enable window display\n";
    }
    
    std::cout << "Resolution: 1440x1080\n";
    std::cout << "Target FPS: 20\n";
    std::cout << "========================================\n\n";
    
    // 如果启用了imshow，尝试创建窗口
    bool window_available = false;
    if (Data::imshow_flag) {
        try {
            cv::namedWindow("Camera Stream - Press q to exit", cv::WINDOW_AUTOSIZE);
            window_available = true;
            std::cout << "[Display] ✓ Window created successfully\n";
        } catch (const cv::Exception& e) {
            std::cout << "[Display] ⚠ Window creation failed: " << e.what() << "\n";
            std::cout << "[Display] ℹ No display server available (headless mode)\n";
            window_available = false;
        } catch (...) {
            std::cout << "[Display] ⚠ Unexpected error creating window\n";
            window_available = false;
        }
    }
    
    int display_fps = 20;
    int frame_delay = 1000 / display_fps;
    
    unsigned long frame_count = 0;
    bool debug_once = true;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (true) {
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            
            if (g_new_frame_available && !g_display_frame.empty()) {
                frame_count++;
                
                if (debug_once) {
                    std::cout << "[Display] ✓ First frame received!\n";
                    std::cout << "  Size: " << g_display_frame.cols << "x" << g_display_frame.rows << "\n";
                    std::cout << "  Channels: " << g_display_frame.channels() << "\n";
                    debug_once = false;
                }
                
                // 显示窗口（如果可用且启用）
                if (window_available && Data::imshow_flag) {
                    try {
                        cv::imshow("Camera Stream - Press q to exit", g_display_frame);
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
                if (frame_count % 100 == 0) {
                    auto current_time = std::chrono::high_resolution_clock::now();
                    double elapsed = std::chrono::duration<double>(current_time - start_time).count();
                    double actual_fps = elapsed > 0 ? frame_count / elapsed : 0;
                    std::cout << "[Display] Processed frames: " << frame_count 
                              << " | FPS: " << std::fixed << std::setprecision(1) << actual_fps << "\n";
                }
                
                g_new_frame_available = false;
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
