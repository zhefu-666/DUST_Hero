#include <iostream>
#include <cstdint>
#include <cstring>

// 模拟实际发送的数据（连续两帧）
void simulate_init_serial_head() {
    const unsigned char sof = 0xA5;
    int struct_size = 16;  // StateBytes size
    int input_size = struct_size + 1;  // = 17
    
    // 模拟接收的34字节数据 (两个完整帧)
    unsigned char header[2 * input_size];  // 34 bytes
    
    // 模拟从串口读取的数据：Frame1 + Frame2
    // Frame1: [0xA5][crc8][12字节data][crc16] = 16字节
    // Frame2: [0xA5][crc8][12字节data][crc16] = 16字节
    memset(header, 0x00, sizeof(header));
    header[0] = 0xA5;   // Frame1 SOF at position 0
    header[16] = 0xA5;  // Frame2 SOF at position 16
    
    std::cout << "=== 调试 initSerialHead 算法 ===" << std::endl;
    std::cout << "struct_size: " << struct_size << std::endl;
    std::cout << "input_size (struct_size + 1): " << input_size << std::endl;
    std::cout << "读取的缓冲区大小: " << 2 * input_size << std::endl;
    std::cout << "\n期望的数据模式:" << std::endl;
    std::cout << "位置0:   Frame1 SOF = 0xA5" << std::endl;
    std::cout << "位置16:  Frame2 SOF = 0xA5" << std::endl;
    
    std::cout << "\n算法查找模式：header[i] == sof && header[i+input_size] == sof" << std::endl;
    std::cout << "这意味着查找：header[i] == 0xA5 && header[i+17] == 0xA5" << std::endl;
    
    std::cout << "\n模拟算法执行：" << std::endl;
    bool found = false;
    int start_index = 0;
    
    for (int i = 0; i < input_size; i++) {
        std::cout << "i=" << i << ": header[" << i << "]=0x" << std::hex << (int)header[i] 
                  << " && header[" << i+input_size << "]=0x" << (int)header[i + input_size];
        
        if (header[i] == sof && header[i + input_size] == sof) {
            found = true;
            start_index = i;
            std::cout << " ✓ 匹配！";
        }
        std::cout << std::dec << std::endl;
    }
    
    std::cout << "\n结果: " << (found ? "✓ 找到同步点" : "✗ 找不到同步点") << std::endl;
    
    if (!found) {
        std::cout << "\n问题分析：" << std::endl;
        std::cout << "算法期望在距离为17的两个位置找到SOF" << std::endl;
        std::cout << "但实际数据中，SOF位于位置0和16" << std::endl;
        std::cout << "距离是16，不是17！" << std::endl;
    }
}

int main() {
    simulate_init_serial_head();
    return 0;
}
