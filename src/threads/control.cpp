#include "garage/garage.h"
#include "threads/control.h"
#include "threads/control/crc.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <cstring>
#include <unistd.h>

using namespace rm;

// 发送计数器
static int send_count = 0;

void Control::send_single(double yaw, double pitch, bool fire, rm::ArmorID id) {
    if (!Data::serial_flag) return;
    
    std::lock_guard<std::mutex> lock(serial_mutex_);  // 保护串口操作
    
    // 检查文件描述符是否有效
    if (file_descriptor_ < 0) {
        rm::message("Serial port not initialized", rm::MSG_ERROR);
        return;
    }
    
    operate_bytes_.frame_header.sof = SOF;
    operate_bytes_.output_data.shoot_yaw = static_cast<float>(yaw);
    operate_bytes_.output_data.shoot_pitch = static_cast<float>(pitch);
    operate_bytes_.output_data.fire = fire;

    #if defined(TJURM_INFANTRY)
    operate_bytes_.output_data.target_id = static_cast<uint8_t>(id);
    #endif

    append_crc16_check_sum((uint8_t*)&operate_bytes_, sizeof(OperateBytes));
    
    // 实时显示发送给下位机的数据
    send_count++;
    
    // 判断是否有有效目标 (id != UNKNOWN 表示检测到了装甲板)
    bool has_target = (id != rm::ARMOR_ID_UNKNOWN);
    
    // 每5帧输出一次
    if (send_count % 5 == 0) {
        if (has_target) {
            // 有目标 - 显示计算出的目标角度（绿色高亮）
            std::cout << "\033[32m[TX#" << send_count << " TARGET] TargetYaw=" << std::fixed << std::setprecision(4) << yaw
                      << " TargetPitch=" << std::setprecision(4) << pitch
                      << " Fire=" << (fire ? "YES" : "NO")
                      << " ArmorID=" << (int)id << "\033[0m" << std::endl;
        } else {
            // 无目标 - 每50帧显示一次云台角度
            if (send_count % 50 == 0) {
                std::cout << "[TX#" << send_count << " IDLE] GimbalYaw=" << std::fixed << std::setprecision(4) << yaw
                          << " GimbalPitch=" << std::setprecision(4) << pitch
                          << " (无目标,发送云台当前角度)" << std::endl;
            }
        }
    }
    
    int status = (int)rm::writeToSerialPort((uint8_t*)&operate_bytes_, sizeof(operate_bytes_), file_descriptor_);
    if (status) {
        rm::message("Control error: " + std::to_string(status), rm::MSG_ERROR);
        // ❌ DO NOT call init_serial() while holding the lock!
        // ❌ DO NOT call restartSerialPort() - let receive_thread handle it
        // Just mark the port as invalid
        file_descriptor_ = -1;
        rm::message("Serial port communication failed, marked as invalid", rm::MSG_WARNING);
    }
}





void Control::autoaim() {
    std::thread send_thread(&Control::send_thread, this);
    send_thread.detach();

    if (Data::serial_flag) {
        std::thread receive_thread(&Control::receive_thread, this);
        receive_thread.detach();
    }
}
