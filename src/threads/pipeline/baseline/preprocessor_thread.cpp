#include "threads/pipeline.h"
#include <atomic>
extern std::atomic<bool> g_running;
#include <unistd.h>
#include <iostream>
#include <openrm/cudatools.h>

using namespace rm;
using namespace nvinfer1;
using namespace nvonnxparser;

void Pipeline::preprocessor_baseline_thread(
    std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out
) {
    auto param = Param::get_instance();
    auto garage = Garage::get_instance();

    std::string yolo_type   = (*param)["Model"]["YoloArmor"]["Type"];
    std::string onnx_file   = (*param)["Model"]["YoloArmor"][yolo_type]["DirONNX"];
    std::string engine_file = (*param)["Model"]["YoloArmor"][yolo_type]["DirEngine"];
    

    int  infer_width     = (*param)["Model"]["YoloArmor"][yolo_type]["InferWidth"];
    int  infer_height    = (*param)["Model"]["YoloArmor"][yolo_type]["InferHeight"];
    int  class_num       = (*param)["Model"]["YoloArmor"][yolo_type]["ClassNum"];
    int  locate_num      = (*param)["Model"]["YoloArmor"][yolo_type]["LocateNum"];
    int  color_num       = (*param)["Model"]["YoloArmor"][yolo_type]["ColorNum"];
    int  bboxes_num      = (*param)["Model"]["YoloArmor"][yolo_type]["BboxesNum"];
    bool hist_input_flag = (*param)["Model"]["YoloArmor"][yolo_type]["NeedHist"];

    std::cout << "[PREPROC] 配置: engine=" << engine_file << std::endl;
    std::cout << "[PREPROC] infer=" << infer_width << "x" << infer_height 
              << " class=" << class_num << " locate=" << locate_num 
              << " bboxes=" << bboxes_num << std::endl;

    if (access(engine_file.c_str(), F_OK) == 0) {
        std::cout << "[PREPROC] 加载引擎文件..." << std::endl;
        if (!rm::initTrtEngine(engine_file, &armor_context_)) {
            std::cerr << "[PREPROC] 引擎加载失败!" << std::endl;
            g_running = false;
            return;
        }
        std::cout << "[PREPROC] 引擎加载成功, context=" << armor_context_ << std::endl;
    } else if (access(onnx_file.c_str(), F_OK) == 0){
        if (!rm::initTrtOnnx(onnx_file, engine_file, &armor_context_, 1U)) {
            g_running = false;
            return;
        }
    } else {
        rm::message("No model file found!", rm::MSG_ERROR);
        g_running = false;
        return;
    }

    size_t yolo_struct_size = sizeof(float) * static_cast<size_t>(locate_num + 1 + color_num + class_num);
    std::cout << "[PREPROC] yolo_struct_size=" << yolo_struct_size << " bytes (" 
              << (locate_num + 1 + color_num + class_num) << " floats)" << std::endl;
    
    mallocYoloDetectBuffer(
        &armor_input_device_buffer_, 
        &armor_output_device_buffer_, 
        &armor_output_host_buffer_, 
        infer_width, 
        infer_height, 
        yolo_struct_size,
        bboxes_num);

    std::cout << "[PREPROC] 缓冲区分配完成:" << std::endl;
    std::cout << "  input_device=" << armor_input_device_buffer_ << std::endl;
    std::cout << "  output_device=" << armor_output_device_buffer_ << std::endl;
    std::cout << "  output_host=" << armor_output_host_buffer_ << std::endl;

    std::mutex mutex;
    TimePoint frame_wait, flag_wait;
    TimePoint tp0, tp1, tp2;
    int preproc_count = 0;
    
    while(g_running) {
        if (!Data::armor_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            while (!Data::armor_mode && g_running) {
                armor_cv_.wait_for(lock, std::chrono::milliseconds(100));
            }
            if (!g_running) break;
        }

        Camera* camera = Data::camera[Data::camera_index];
        std::shared_ptr<rm::Frame> frame = camera->buffer->pop();

        frame_wait = tp1 = getTime();
        while(frame == nullptr && g_running) {
            frame = camera->buffer->pop();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            double delay = getDoubleOfS(frame_wait, getTime());
            if (delay > 2.0 && Data::timeout_flag) {
                rm::message("Capture timeout warning", rm::MSG_WARNING);
                frame_wait = getTime();  // 重置计时器，继续等待
            }
        }
        if (!g_running) break;
        if (frame == nullptr) continue;

        preproc_count++;
        
        // 调试: 检查输入图像
        if (preproc_count <= 5) {
            std::cout << "[PREPROC] 帧 " << preproc_count 
                      << ": " << frame->width << "x" << frame->height 
                      << " image=" << (frame->image ? "有效" : "空") << std::endl;
            if (frame->image && !frame->image->empty()) {
                cv::Mat& img = *frame->image;
                double minVal, maxVal;
                cv::minMaxLoc(img.reshape(1), &minVal, &maxVal);
                std::cout << "[PREPROC] 图像范围: min=" << minVal << " max=" << maxVal << std::endl;
            }
        }

        memcpyYoloCameraBuffer(
            frame->image->data,
            camera->rgb_host_buffer,
            camera->rgb_device_buffer,
            frame->width,
            frame->height);
        
        resize(
            camera->rgb_device_buffer,
            frame->width,
            frame->height,
            armor_input_device_buffer_,
            infer_width,
            infer_height,
            (void*)resize_stream_
        );
        
        cudaStreamSynchronize(resize_stream_);
        
        detectEnqueue(
            armor_input_device_buffer_,
            armor_output_device_buffer_,
            &armor_context_,
            &detect_stream_
        );

        if (Data::record_mode) { record(frame); }

        tp2 = getTime();
        if (Data::pipeline_delay_flag) rm::message("preprocess", getDoubleOfS(tp1, tp2) * 1000);

        flag_wait = getTime();
        while(flag_out && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (getDoubleOfS(flag_wait, getTime()) > 10.0 && Data::timeout_flag) {
                rm::message("Preprocessor timeout warning", rm::MSG_WARNING);
                flag_wait = getTime();  // 重置计时器，继续等待
            }
        }
        if (!g_running) break;
        
        std::unique_lock<std::mutex> lock_out(mutex_out);
        frame_out = frame;
        flag_out = true;
    }
    std::cout << "[preprocessor_thread] Exiting..." << std::endl;
}
