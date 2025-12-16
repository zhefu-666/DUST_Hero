#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <vector>

int setup_serial_port(const char* port_name, int baudrate) {
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void print_hex_data(const unsigned char* data, int len) {
    for (int i = 0; i < len; i++) {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)data[i] << " ";
    }
    std::cout << std::endl;
}

void analyze_frame_structure(const unsigned char* data, int len) {
    std::cout << "=============== FRAME ANALYSIS ===============" << std::endl;
    std::cout << "Total bytes: " << std::dec << len << std::endl;
    std::cout << std::endl;
    
    // 显示完整的十六进制数据，带索引
    std::cout << "Index  Hex    Char  Description" << std::endl;
    std::cout << "-----  -----  ----  -----------" << std::endl;
    
    for (int i = 0; i < len; i++) {
        std::cout << std::dec << std::setfill(' ') << std::setw(2) << i << ":     ";
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)data[i] << "     ";
        
        // 显示ASCII字符
        if (data[i] >= 32 && data[i] < 127) {
            std::cout << (char)data[i];
        } else {
            std::cout << ".";
        }
        
        std::cout << "      ";
        
        // 描述
        if (i == 0) {
            std::cout << "Byte 0: Frame Header (SOF)";
        } else if (i == 1) {
            std::cout << "Byte 1: CRC8 or Header Byte 2?";
        } else if (i == len - 2) {
            std::cout << "Byte N-1: CRC16 Low?";
        } else if (i == len - 1) {
            std::cout << "Byte N: CRC16 High? / Frame Tail";
        }
        
        std::cout << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "Frame Head Analysis:" << std::endl;
    std::cout << "  Byte[0] = 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[0];
    if (data[0] == 0xA5) {
        std::cout << " (Expected SOF=0xA5) ✓" << std::endl;
    } else if (data[0] == 0x53) {
        std::cout << " (NOT 0xA5, actual SOF is 0x53) ✗" << std::endl;
    } else {
        std::cout << " (Unexpected value) ✗" << std::endl;
    }
    
    std::cout << "  Byte[1] = 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[1] << std::endl;
    
    if (len >= 3) {
        std::cout << "  Byte[2] = 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[2] << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "Frame Tail Analysis:" << std::endl;
    if (len >= 2) {
        std::cout << "  Byte[N-1] = 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[len-2] << std::endl;
        std::cout << "  Byte[N]   = 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[len-1];
        
        if (data[len-1] == 0x0A) {
            std::cout << " (Newline '\\n') - Possible frame tail marker" << std::endl;
        } else if (data[len-1] == 0x0D) {
            std::cout << " (Carriage return '\\r') - Possible frame tail marker" << std::endl;
        } else {
            std::cout << std::endl;
        }
    }
    
    // 可能的帧大小分析
    std::cout << std::endl;
    std::cout << "Possible Frame Sizes:" << std::endl;
    std::cout << "  If header=1B, data=15B, crc16=2B, tail=1B -> Total=19B" << std::endl;
    std::cout << "  If header=1B, data=?B, crc16=2B, tail=1B -> Total=" << std::dec << len << "B" << std::endl;
    std::cout << "  If header=2B, data=?B, crc16=2B -> Total=" << std::dec << len << "B" << std::endl;
    
    std::cout << "==========================================" << std::endl;
}

int main() {
    const char* port = "/dev/ttyACM0";
    
    std::cout << "Opening serial port: " << port << std::endl;
    int fd = setup_serial_port(port, 115200);
    
    if (fd < 0) {
        return 1;
    }
    
    std::cout << "Serial port opened successfully" << std::endl;
    std::cout << "Waiting for data... (Press Ctrl+C to exit)" << std::endl;
    std::cout << std::endl;
    
    unsigned char buffer[1024];
    int packet_count = 0;
    
    while (1) {
        memset(buffer, 0, sizeof(buffer));
        int bytes_read = read(fd, buffer, sizeof(buffer));
        
        if (bytes_read > 0) {
            packet_count++;
            std::cout << "\n" << std::string(60, '=') << std::endl;
            std::cout << "Packet #" << packet_count << std::endl;
            std::cout << std::string(60, '=') << std::endl;
            
            analyze_frame_structure(buffer, bytes_read);
        }
        
        usleep(100000);  // 100ms延迟
    }
    
    close(fd);
    return 0;
}
