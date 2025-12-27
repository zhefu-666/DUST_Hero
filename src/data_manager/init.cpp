#include <string>
#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <chrono>
#include <mutex>
#include "data_manager/base.h"
#include "data_manager/param.h"
#include "threads/pipeline.h"
#include "threads/control.h"
#include "garage/garage.h"
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

using namespace std;

// 全局相机句柄和帧缓冲
static void* g_camera_handle = NULL;
std::mutex g_frame_mutex;
cv::Mat g_display_frame;
bool g_new_frame_available = false;

// 视频录制器
static cv::VideoWriter* g_video_writer = nullptr;
static std::mutex g_writer_mutex;
static int g_frame_count = 0;
static std::chrono::high_resolution_clock::time_point g_start_time;
static bool g_recording = false;

// 图像回调函数
void __stdcall HikCameraCallback(unsigned char* pData, MV_FRAME_OUT_INFO* pFrameInfo, void* pUser) {
    if (pData == NULL || pFrameInfo == NULL) return;
    
    try {
        // 将Bayer RG原始数据转换为BGR图像
        cv::Mat raw_image(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
        cv::Mat bgr_image;
        cv::cvtColor(raw_image, bgr_image, cv::COLOR_BayerBG2BGR);  // 海康相机通常使用BayerBG
        
        // 创建 Frame 并推送到缓冲区（供预处理线程使用）
        if (Data::camera.size() > 0 && Data::camera[0] != nullptr && Data::camera[0]->buffer != nullptr) {
            auto frame = std::make_shared<rm::Frame>();
            frame->image = std::make_shared<cv::Mat>();
            bgr_image.copyTo(*(frame->image));
            frame->time_point = getTime();
            frame->camera_id = 0;
            frame->width = pFrameInfo->nWidth;
            frame->height = pFrameInfo->nHeight;
            Data::camera[0]->buffer->push(frame);
        }
        
        // 保存到全局显示缓冲
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            bgr_image.copyTo(g_display_frame);
            g_new_frame_available = true;
        }
        
        // 保存视频
        if (g_recording) {
            std::lock_guard<std::mutex> lock(g_writer_mutex);
            if (g_video_writer && g_video_writer->isOpened()) {
                g_video_writer->write(bgr_image);
                g_frame_count++;
                
                // 检查是否需要停止（30秒）
                auto current_time = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - g_start_time).count();
                
                if (elapsed > 30) {
                    g_video_writer->release();
                    rm::message("Video recording completed. Frames: " + std::to_string(g_frame_count) + 
                                ", Duration: " + std::to_string(elapsed) + "s", rm::MSG_NOTE);
                    g_recording = false;
                }
            }
        }
    } catch (const std::exception& e) {
        rm::message("Error in camera callback: " + std::string(e.what()), rm::MSG_ERROR);
    }
}

void init_debug() {
    auto param = Param::get_instance();
    Data::auto_fire = (*param)["Debug"]["System"]["AutoFire"];
    Data::auto_enemy = (*param)["Debug"]["System"]["AutoEnemy"];
    Data::auto_rune = (*param)["Debug"]["System"]["AutoRune"];
    Data::auto_capture = (*param)["Debug"]["System"]["AutoCapture"];

    Data::plus_pnp = (*param)["Debug"]["PlusPnP"]["Enable"];

    Data::serial_flag = (*param)["Debug"]["Control"]["Serial"];
    Data::timeout_flag = (*param)["Debug"]["Control"]["Timeout"];
    Data::manu_capture = (*param)["Debug"]["Control"]["ManuCapture"];
    Data::manu_fire = (*param)["Debug"]["Control"]["ManuFire"];

    Data::manu_rune = (*param)["Debug"]["Control"]["ManuRune"];
    Data::big_rune = (*param)["Debug"]["Control"]["BigRune"];

    Data::ui_flag = (*param)["Debug"]["ImageThread"]["UI"];
    Data::imwrite_flag = (*param)["Debug"]["ImageThread"]["Imwrite"];
    Data::image_flag = (bool)(Data::imwrite_flag || Data::imshow_flag);
    Data::binary_flag = (*param)["Debug"]["ImageThread"]["Binary"];
    Data::histogram_flag = (*param)["Debug"]["ImageThread"]["Histogram"];

    Data::reprojection_flag = (*param)["Debug"]["Display"]["Reprojection"];
    Data::pipeline_delay_flag = (*param)["Debug"]["Display"]["PipelineDelay"];
    Data::point_skip_flag = (*param)["Debug"]["Display"]["PointSkip"];

    Data::state_delay_flag = (*param)["Debug"]["StateDelay"]["Enable"];
    Data::state_delay_time = (*param)["Debug"]["StateDelay"]["TimeS"];
    Data::state_queue_size = (*param)["Debug"]["StateDelay"]["QueueSize"];
    Data::send_wait_time = (*param)["Debug"]["StateDelay"]["SendWait"];
}

bool init_camera() {
    auto param = Param::get_instance();
    auto control = Control::get_instance();

    // 获取相机参数矩阵json
    nlohmann::json camlens;
    std::string camlen_path = (*param)["Camera"]["CamLensDir"];
    try {
        std::ifstream camlens_json(camlen_path);
        camlens_json >> camlens;
        camlens_json.close();
    } catch (std::exception& e) {
        std::string err_str = "Failed to load CamLens json: " + std::string(e.what());
        rm::message(err_str, rm::MSG_ERROR);
        return false;
    }

    // 枚举海康设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
        rm::message("No Hikvision camera found", rm::MSG_ERROR);
        return false;
    }
    
    int camera_num = stDeviceList.nDeviceNum;
    rm::message("Found " + std::to_string(camera_num) + " Hikvision camera(s)", rm::MSG_NOTE);
    
    Data::camera.clear();
    Data::camera.resize(camera_num, nullptr);
    
    if (camera_num == 1) {
        Data::camera_index = 0;
        Data::camera_base = 0;
        Data::camera_far = 0;

        double exp = (*param)["Camera"]["Base"]["ExposureTime"];
        double gain = (*param)["Camera"]["Base"]["Gain"];
        double rate = (*param)["Camera"]["Base"]["FrameRate"];
        std::string camera_type = (*param)["Camera"]["Base"]["CameraType"];
        std::string lens_type = (*param)["Camera"]["Base"]["LensType"];
        std::vector<double> camera_offset = (*param)["Car"]["CameraOffset"]["Base"];

        Data::camera[0] = new rm::Camera();
        Data::camera[0]->buffer = new rm::SwapBuffer<rm::Frame>();
        
        // 创建相机句柄
        void* handle = NULL;
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
        if (MV_OK != nRet) {
            rm::message("Failed to create camera handle", rm::MSG_ERROR);
            delete Data::camera[0];
            Data::camera[0] = nullptr;
            return false;
        }
        
        // 打开相机
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet) {
            rm::message("Failed to open Hikvision camera", rm::MSG_ERROR);
            MV_CC_DestroyHandle(handle);
            delete Data::camera[0];
            Data::camera[0] = nullptr;
            return false;
        }
        
        rm::message("Hikvision camera opened successfully", rm::MSG_NOTE);
        g_camera_handle = handle;
        
        // 设置曝光模式为手动
        MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
        
        // 设置曝光时间
        MV_CC_SetFloatValue(handle, "ExposureTime", exp);
        
        // 设置增益（0-17范围）
        if (gain > 17.0) gain = 17.0;
        MV_CC_SetFloatValue(handle, "Gain", gain);
        
        // 设置帧率
        MV_CC_SetFloatValue(handle, "ResultingFrameRate", rate);
        
        // 获取相机分辨率
        MVCC_INTVALUE width_value = {0};
        MVCC_INTVALUE height_value = {0};
        MV_CC_GetIntValue(handle, "Width", &width_value);
        MV_CC_GetIntValue(handle, "Height", &height_value);
        
        // 使用硬编码的分辨率（海康MV-CS016-10UC为1440x1080）
        int width = 1440;
        int height = 1080;
        
        if (width_value.nCurValue > 0 && width_value.nCurValue < 10000) {
            width = width_value.nCurValue;
        }
        if (height_value.nCurValue > 0 && height_value.nCurValue < 10000) {
            height = height_value.nCurValue;
        }
        
        Data::camera[0]->width = width;
        Data::camera[0]->height = height;
        
        rm::message("Camera resolution: " + std::to_string(width) + "x" + std::to_string(height), rm::MSG_NOTE);
        
        // 注册图像回调
        nRet = MV_CC_RegisterImageCallBack(handle, HikCameraCallback, NULL);
        if (MV_OK != nRet) {
            rm::message("Failed to register image callback", rm::MSG_ERROR);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            delete Data::camera[0];
            Data::camera[0] = nullptr;
            return false;
        }
        
        // 开始取流
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet) {
            rm::message("Failed to start camera grabbing", rm::MSG_ERROR);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            delete Data::camera[0];
            Data::camera[0] = nullptr;
            return false;
        }
        
        rm::message("Camera grabbing started successfully", rm::MSG_NOTE);
        
        // 加载标定参数
        Param::from_json(camlens[camera_type][lens_type]["Intrinsic"], Data::camera[0]->intrinsic_matrix);
        Param::from_json(camlens[camera_type][lens_type]["Distortion"], Data::camera[0]->distortion_coeffs);
        
        // 验证内参矩阵是否成功加载
        std::cout << "[CAMERA-INIT] Intrinsic matrix loaded: " << Data::camera[0]->intrinsic_matrix.rows 
                  << "x" << Data::camera[0]->intrinsic_matrix.cols << std::endl;
        if (Data::camera[0]->intrinsic_matrix.rows == 3 && Data::camera[0]->intrinsic_matrix.cols == 3) {
            double fx = Data::camera[0]->intrinsic_matrix.at<double>(0, 0);
            double fy = Data::camera[0]->intrinsic_matrix.at<double>(1, 1);
            double cx = Data::camera[0]->intrinsic_matrix.at<double>(0, 2);
            double cy = Data::camera[0]->intrinsic_matrix.at<double>(1, 2);
            std::cout << "[CAMERA-INIT] fx=" << fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << std::endl;
        }
        
        
        // 验证内参矩阵是否成功加载
        std::cout << "[CAMERA-INIT] Intrinsic matrix loaded: " << Data::camera[0]->intrinsic_matrix.rows 
                  << "x" << Data::camera[0]->intrinsic_matrix.cols << std::endl;
        if (Data::camera[0]->intrinsic_matrix.rows == 3 && Data::camera[0]->intrinsic_matrix.cols == 3) {
            double fx = Data::camera[0]->intrinsic_matrix.at<double>(0, 0);
            double fy = Data::camera[0]->intrinsic_matrix.at<double>(1, 1);
            double cx = Data::camera[0]->intrinsic_matrix.at<double>(0, 2);
            double cy = Data::camera[0]->intrinsic_matrix.at<double>(1, 2);
            std::cout << "[CAMERA-INIT] fx=" << fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << std::endl;
        }
        
        rm::tf_rotate_pnp2head(Data::camera[0]->Rotate_pnp2head, camera_offset[3], camera_offset[4], 0.0);
        rm::tf_trans_pnp2head(Data::camera[0]->Trans_pnp2head, camera_offset[0], camera_offset[1], 
                            camera_offset[2], camera_offset[3], camera_offset[4], 0.0);
        rm::mallocYoloCameraBuffer(&Data::camera[0]->rgb_host_buffer, &Data::camera[0]->rgb_device_buffer, 
                                  width, height);
        
        // 启动视频录制（如果启用imshow_flag）
        if (Data::imshow_flag) {
            system("mkdir -p /home/hero/TJURM-2024/data/video");
            std::string video_path = "/home/hero/TJURM-2024/data/video/camera_stream_" + 
                                    std::to_string(std::time(nullptr)) + ".avi";
            
            {
                std::lock_guard<std::mutex> lock(g_writer_mutex);
                g_video_writer = new cv::VideoWriter(video_path, 
                                                     cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                                     30, 
                                                     cv::Size(width, height));
                
                if (g_video_writer->isOpened()) {
                    g_recording = true;
                    g_frame_count = 0;
                    g_start_time = std::chrono::high_resolution_clock::now();
                    rm::message("Video recording started: " + video_path, rm::MSG_NOTE);
                } else {
                    rm::message("Failed to open video writer", rm::MSG_ERROR);
                    delete g_video_writer;
                    g_video_writer = nullptr;
                }
            }
        }
        
        rm::message("Camera initialized successfully", rm::MSG_NOTE);
        return true;
        
    } else {
        rm::message("Multi-camera mode not fully supported yet", rm::MSG_WARNING);
        return false;
    }
}

bool deinit_camera() {
    // 首先停止相机采集 - 这会停止回调函数被调用
    if (g_camera_handle != nullptr) {
        MV_CC_StopGrabbing(g_camera_handle);
        // 等待一小段时间，确保回调完成
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        MV_CC_CloseDevice(g_camera_handle);
        MV_CC_DestroyHandle(g_camera_handle);
        g_camera_handle = nullptr;
    }
    
    // 关闭视频写入器
    {
        std::lock_guard<std::mutex> lock(g_writer_mutex);
        if (g_video_writer) {
            g_video_writer->release();
            delete g_video_writer;
            g_video_writer = nullptr;
        }
        g_recording = false;
    }
    
    // 现在可以安全地释放相机资源
    for(int i = 0; i < Data::camera.size(); i++) {
        if(Data::camera[i] == nullptr) continue;
        
        // 释放YOLO缓冲
        if (Data::camera[i]->rgb_host_buffer != nullptr || Data::camera[i]->rgb_device_buffer != nullptr) {
            rm::freeYoloCameraBuffer(Data::camera[i]->rgb_host_buffer, Data::camera[i]->rgb_device_buffer);
            Data::camera[i]->rgb_host_buffer = nullptr;
            Data::camera[i]->rgb_device_buffer = nullptr;
        }

        delete Data::camera[i];
        Data::camera[i] = nullptr;
    }
    
    rm::message("Camera deinit success", rm::MSG_WARNING);
    return true;
}


void init_serial() {
    int status;
    std::vector<std::string> port_list;
    auto control = Control::get_instance();

    // 获取互斥锁以保护串口操作
    std::lock_guard<std::mutex> lock(control->serial_mutex_);

    // 关闭已打开的文件描述符（如果存在）
    if (control->file_descriptor_ > 0) {
        rm::closeSerialPort(control->file_descriptor_);
        control->file_descriptor_ = -1;
    }

    while(true) {

        #if defined(TJURM_HERO)
        status = (int)rm::getSerialPortList(port_list, rm::SERIAL_TYPE_TTY_ACM);
        #endif

        #if defined(TJURM_BALANCE) || defined(TJURM_INFANTRY) || defined(TJURM_DRONSE) || defined(TJURM_SENTRY)
        status = (int)rm::getSerialPortList(port_list, rm::SERIAL_TYPE_TTY_USB);
        #endif

        if (status != 0 || port_list.empty()) {
            rm::message("Control port list failed", rm::MSG_ERROR);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            port_list.clear();
            continue;
        }

        control->port_name_ = port_list[0];
        status = (int)rm::openSerialPort(control->file_descriptor_, control->port_name_);
        if (status != 0) {
            rm::message("Control port open failed", rm::MSG_ERROR);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            port_list.clear();
            continue;
        }
        if(status == 0) {
            rm::message("Serial port initialized successfully: " + control->port_name_, rm::MSG_WARNING);
            break;
        }
    }
}






void init_attack() {
    #ifdef TJURM_SENTRY
    Data::attack = new rm::Filtrate();
    #endif

    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_DRONSE)
    Data::attack = new rm::DeadLocker();
    #endif 
}
