#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>

// 设置串口参数
int setup_serial_port(const char* port_name, int baudrate) {
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 设置数据位、停止位、校验位
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;      // 无校验位
    options.c_cflag &= ~CSTOPB;      // 1个停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;           // 8个数据位

    // 原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// 打印十六进制数据
void print_hex_data(const unsigned char* data, int len) {
    for (int i = 0; i < len; i++) {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)data[i] << " ";
    }
    std::cout << std::endl;
}

// 打印数据分析
void analyze_data(const unsigned char* data, int len) {
    std::cout << "=============== DATA ANALYSIS ===============" << std::endl;
    std::cout << "Total bytes: " << len << std::endl;
    
    if (len > 0) {
        std::cout << "Frame Header (byte 0): 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[0];
        if (data[0] == 0xA5) {
            std::cout << " ✓ (SOF = 0xA5)" << std::endl;
        } else {
            std::cout << " ✗ (Expected 0xA5)" << std::endl;
        }
    }
    
    if (len > 1) {
        std::cout << "CRC8 (byte 1): 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[1] << std::endl;
    }
    
    if (len > 2) {
        std::cout << "Data bytes (" << std::dec << (len - 4) << " bytes): ";
        if (len > 3) {
            print_hex_data(data + 2, len - 4);
        }
    }
    
    if (len >= 2) {
        std::cout << "CRC16 (last 2 bytes): 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[len-2] 
                  << " 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data[len-1] << std::endl;
    }
    
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
            std::cout << "\n--- Packet #" << packet_count << " ---" << std::endl;
            std::cout << "Hex dump: ";
            print_hex_data(buffer, bytes_read);
            analyze_data(buffer, bytes_read);
        }
        
        usleep(100000);  // 100ms延迟
    }
    
    close(fd);
    return 0;
}
