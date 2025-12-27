#include "threads/control.h"
#include "threads/control/crc.h"

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <iomanip>
#include <sstream>

using namespace rm;

static rm::SerialStatus custom_init_serial_head(int& fd, size_t struct_size, unsigned char sof) {
    unsigned char buffer[2 * struct_size];
    
    if (rm::readFromSerialPort(buffer, 2 * struct_size, fd) != rm::SERIAL_STATUS_OK) {
        rm::message("Serial port read failed", rm::MSG_ERROR);
        return rm::SERIAL_STATUS_INIT_HEAD_FAILED;
    }
    
    bool found = false;
    int start_index = 0;
    
    for (int i = 0; i < (int)struct_size; i++) {
        if (buffer[i] == sof && buffer[i + struct_size] == sof) {
            found = true;
            start_index = i;
            break;
        }
    }
    
    if (!found) {
        return rm::SERIAL_STATUS_INIT_HEAD_FAILED;
    }
    
    if (start_index > 0) {
        unsigned char discard[start_index];
        rm::readFromSerialPort(discard, start_index, fd);
    }
    
    return rm::SERIAL_STATUS_OK;
}

void Control::receive_thread() {
    static int frame_count = 0;
    static int crc_pass_count = 0;
    
    std::cout << "\n========================================\n";
    std::cout << "Serial Port Communication\n";
    std::cout << "Port: " << port_name_ << "\n";
    std::cout << "Frame Size: " << sizeof(StateBytes) << " bytes\n";
    std::cout << "========================================\n\n";
    
    custom_init_serial_head(file_descriptor_, sizeof(StateBytes), SOF);
    
    char buffer[400];
    StateBytes state_buffer;

    int status = 0;
    TimePoint tp = getTime();
    while (true) {
        double dt = getDoubleOfS(tp, getTime());
        // 超时检查已禁用 - 程序将持续运行
        // if(dt > 1.0 && Data::timeout_flag) {
        //     rm::message("Control error: timeout", rm::MSG_ERROR);
        //     exit(-1);
        // }

        if(status != 0) {
            rm::message("Control error: " + std::to_string(status), rm::MSG_ERROR);
            if (access(port_name_.c_str(), F_OK) < 0) {
                init_serial();
                status = 0;
            } else {
                status = (int)rm::restartSerialPort(file_descriptor_, port_name_);
            }
        }

        memset(buffer, 0, sizeof(buffer));
        status = (int)rm::readFromSerialPort((uint8_t*)buffer, sizeof(StateBytes), file_descriptor_);
        if (status) {
            rm::message("Control error: " + std::to_string(status), rm::MSG_ERROR);
            continue;
        }
        
        frame_count++;
        
        if ((unsigned char)buffer[0] != SOF) {
            if (frame_count % 50 == 0) {
                std::cout << "[#" << frame_count << "] Bad SOF: 0x" << std::hex << (int)(unsigned char)buffer[0] << std::dec << "\n";
            }
            status = (int)custom_init_serial_head(file_descriptor_, sizeof(StateBytes), SOF);
            continue;
        }

        bool crc_ok = verify_crc16_check_sum((uint8_t*)buffer, sizeof(StateBytes));
        if (!crc_ok) {
            continue;
        }
        
        crc_pass_count++;

        // 使用结构体方法
        memcpy(&state_buffer, buffer, sizeof(StateBytes));

        if (Data::state_delay_flag) {
            std::pair<TimePoint, StateBytes> p = std::make_pair(getTime(), state_buffer);
            state_queue_.push_back(p);
            if (state_queue_.size() > Data::state_queue_size) {
                state_queue_.pop_front();
            }

            auto it = state_queue_.begin();
            for (; it != state_queue_.end(); ++it) {
                if (getDoubleOfS(it->first, getTime()) < Data::state_delay_time) {
                    this->state_bytes_ = it->second;
                    break;
                }
            }
            if (it == state_queue_.end()) {
                rm::message("Control error: state delay error", rm::MSG_ERROR);
                this->state_bytes_ = state_buffer;
            }

        } else {
            this->state_bytes_ = state_buffer;
        }

        Data::yaw   = get_yaw();
        Data::pitch = get_pitch();
        Data::roll  = get_roll();

        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
        Data::yaw_omega = get_yaw_omega();
        #endif

        tp = getTime();
    }
}
