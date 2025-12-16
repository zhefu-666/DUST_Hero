#include <iostream>
#include <cstdint>

struct FrameHeader {
    uint8_t     sof;
    uint8_t     crc8;
} __attribute__((packed));

struct FrameTailer {
    uint16_t    crc16;
} __attribute__((packed));

struct InputData {
    float       curr_yaw;
    float       curr_pitch;
    uint8_t     state;
    uint8_t     autoaim;
    uint8_t     enemy_color;
    uint8_t     reserved;
} __attribute__((packed));

struct StateBytes {
    FrameHeader frame_header;
    InputData   input_data;
    FrameTailer frame_tailer;   
} __attribute__((packed));

int main() {
    std::cout << "sizeof(FrameHeader): " << sizeof(FrameHeader) << std::endl;
    std::cout << "sizeof(InputData): " << sizeof(InputData) << std::endl;
    std::cout << "sizeof(FrameTailer): " << sizeof(FrameTailer) << std::endl;
    std::cout << "sizeof(StateBytes): " << sizeof(StateBytes) << std::endl;
    
    // 模拟initSerialHead算法
    int struct_size = sizeof(StateBytes);
    int input_size = struct_size + 1;
    std::cout << "\nstruct_size: " << struct_size << std::endl;
    std::cout << "input_size (struct_size + 1): " << input_size << std::endl;
    std::cout << "buffer size for 2*input_size: " << 2*input_size << std::endl;
    std::cout << "\nThe algorithm looks for SOF at positions [i] and [i+" << input_size << "]" << std::endl;
    
    return 0;
}
