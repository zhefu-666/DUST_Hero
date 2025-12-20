#include "threads/pipeline.h"
#include <atomic>
extern std::atomic<bool> g_running;
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <openrm/cudatools.h>
using namespace rm;
using namespace nvinfer1;
using namespace nvonnxparser;

// 外部声明 - 更新显示线程的检测结果
extern void update_global_detections(const std::vector<rm::YoloRect>& detections);

void Pipeline::detector_baseline_thread(
    std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in, 
    std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out
) {
    auto param = Param::get_instance();
    auto garage = Garage::get_instance();

    std::string yolo_type = (*param)["Model"]["YoloArmor"]["Type"];

    int    infer_width       = (*param)["Model"]["YoloArmor"][yolo_type]["InferWidth"];
    int    infer_height      = (*param)["Model"]["YoloArmor"][yolo_type]["InferHeight"];
    int    class_num         = (*param)["Model"]["YoloArmor"][yolo_type]["ClassNum"];
    int    locate_num        = (*param)["Model"]["YoloArmor"][yolo_type]["LocateNum"];
    int    color_num         = (*param)["Model"]["YoloArmor"][yolo_type]["ColorNum"];
    int    bboxes_num        = (*param)["Model"]["YoloArmor"][yolo_type]["BboxesNum"];
    double confidence_thresh = (*param)["Model"]["YoloArmor"][yolo_type]["ConfThresh"];
    double nms_thresh        = (*param)["Model"]["YoloArmor"][yolo_type]["NMSThresh"];

    size_t yolo_struct_size = sizeof(float) * static_cast<size_t>(locate_num + 1 + color_num + class_num);
    int struct_len = locate_num + 1 + color_num + class_num;
    
    std::cout << "[DETECTOR] 启动检测线程" << std::endl;
    std::cout << "[DETECTOR] Type=" << yolo_type << " struct_len=" << struct_len << std::endl;
    std::cout << "[DETECTOR] confidence_thresh=" << confidence_thresh << std::endl;
    
    std::mutex mutex;
    int debug_counter = 0;
    
    while (g_running) {
        // 等待 armor_mode 激活
        if (!Data::armor_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            while (!Data::armor_mode && g_running) {
                armor_cv_.wait_for(lock, std::chrono::milliseconds(100));
            }
            if (!g_running) break;
        }

        // 等待输入帧
        while(!flag_in && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (!g_running) break;
        
        std::unique_lock<std::mutex> lock_in(mutex_in);
        if (!flag_in) {
            lock_in.unlock();
            continue;
        }
        std::shared_ptr<rm::Frame> frame = frame_in;
        flag_in = false;
        lock_in.unlock();

        detectOutput(
            armor_output_host_buffer_,
            armor_output_device_buffer_,
            &detect_stream_,
            yolo_struct_size,
            bboxes_num
        );
        
        debug_counter++;
        
        // 调用NMS获取检测结果
        if (yolo_type == "V5") {
            frame->yolo_list = yoloArmorNMS_V5(
                armor_output_host_buffer_,
                bboxes_num,
                class_num,
                confidence_thresh,
                nms_thresh,
                frame->width,
                frame->height,
                infer_width,
                infer_height
            );
        } else if (yolo_type == "FP") {
            frame->yolo_list = yoloArmorNMS_FP(
                armor_output_host_buffer_,
                bboxes_num,
                class_num,
                confidence_thresh,
                nms_thresh,
                frame->width,
                frame->height,
                infer_width,
                infer_height
            );
        } else if (yolo_type == "FPX") {
            frame->yolo_list = yoloArmorNMS_FPX(
                armor_output_host_buffer_,
                bboxes_num,
                class_num,
                confidence_thresh,
                nms_thresh,
                frame->width,
                frame->height,
                infer_width,
                infer_height
            );
        } else {
            rm::message("Invalid yolo type", rm::MSG_ERROR);
            g_running = false;
            break;
        }
        
        // 更新全局检测结果供显示线程使用
        update_global_detections(frame->yolo_list);
        
        // 调试输出（每30帧）
        if (debug_counter % 30 == 1) {
            std::cout << "[DETECT] 帧 " << debug_counter 
                      << " | 检测数量: " << frame->yolo_list.size();
            if (!frame->yolo_list.empty()) {
                std::cout << " | 第一个: class=" << frame->yolo_list[0].class_id
                          << " conf=" << frame->yolo_list[0].confidence;
            }
            std::cout << std::endl;
        }

        std::unique_lock<std::mutex> lock_out(mutex_out);
        frame_out = frame;
        flag_out = true;
        lock_out.unlock();
    }
    std::cout << "[detector_thread] Exiting..." << std::endl;
}
