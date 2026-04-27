// serial.cpp
#include "serial.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <arpa/inet.h>

// ==================== 构造函数和析构函数 ====================
SerialPort::SerialPort() : serial_fd(-1), baud_rate(115200) {
}

SerialPort::~SerialPort() {
    close();
}

// ==================== 串口基本操作 ====================
bool SerialPort::open(const std::string& port_name, int baud_rate) {
    // 如果已经打开，先关闭
    if (serial_fd >= 0) {
        close();
    }
    
    this->port_name = port_name;
    this->baud_rate = baud_rate;
    
    // 打开串口设备
    serial_fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        std::cerr << "Error: Failed to open serial port " << port_name 
                  << " (" << strerror(errno) << ")" << std::endl;
        return false;
    }
    
    // 清除O_NONBLOCK标志，使用阻塞模式
    int flags = fcntl(serial_fd, F_GETFL, 0);
    fcntl(serial_fd, F_SETFL, flags & ~O_NONBLOCK);
    
    // 配置串口参数
    if (!configureSerialPort()) {
        ::close(serial_fd);
        serial_fd = -1;
        return false;
    }
    
    // 清空输入输出缓冲区
    tcflush(serial_fd, TCIOFLUSH);
    
    std::cout << "Success: Serial port " << port_name 
              << " opened (baud rate: " << baud_rate << ")" << std::endl;
    return true;
}

bool SerialPort::send(const uint8_t* data, size_t length) {
    if (serial_fd < 0) {
        std::cerr << "Error: Serial port not opened!" << std::endl;
        return false;
    }
    
    if (length == 0) {
        std::cerr << "Warning: Attempting to send empty data!" << std::endl;
        return false;
    }
    
    ssize_t bytes_written = write(serial_fd, data, length);
    if (bytes_written < 0) {
        std::cerr << "Error: Failed to write to serial port (" 
                  << strerror(errno) << ")" << std::endl;
        return false;
    }
    
    if (bytes_written != static_cast<ssize_t>(length)) {
        std::cerr << "Error: Only wrote " << bytes_written 
                  << " of " << length << " bytes!" << std::endl;
        return false;
    }
    
    // 确保数据发送完成
    tcdrain(serial_fd);
    return true;
}

size_t SerialPort::receive(uint8_t* buffer, size_t buffer_size) {
    if (serial_fd < 0) {
        std::cerr << "Error: Serial port not opened!" << std::endl;
        return 0;
    }
    
    if (buffer_size == 0) {
        return 0;
    }
    
    ssize_t bytes_read = read(serial_fd, buffer, buffer_size);
    if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Error: Failed to read from serial port (" 
                      << strerror(errno) << ")" << std::endl;
        }
        return 0;
    }
    
    return static_cast<size_t>(bytes_read);
}

void SerialPort::close() {
    if (serial_fd >= 0) {
        ::close(serial_fd);
        serial_fd = -1;
        std::cout << "Serial port closed: " << port_name << std::endl;
    }
}

bool SerialPort::isOpen() const {
    return serial_fd >= 0;
}

// ==================== 速度控制相关功能 ====================
void SerialPort::createVelocityCommand(float vx, float vy, float wz, uint8_t* buffer, size_t& length) {
    int index = 0;
    
    // 帧头
    buffer[index++] = 0xFE;  // 字节0
    buffer[index++] = 0xEF;  // 字节1
    
    // 数据长度：固定为13字节
    buffer[index++] = 0x0D;  // 字节2
    
    // 帧功能码
    buffer[index++] = 0x01;  // 字节3
    
    // X轴速度 (float转网络字节序)
    uint32_t vx_net = htonl(*reinterpret_cast<uint32_t*>(&vx));
    memcpy(&buffer[index], &vx_net, 4);
    index += 4;
    
    // Y轴速度
    uint32_t vy_net = htonl(*reinterpret_cast<uint32_t*>(&vy));
    memcpy(&buffer[index], &vy_net, 4);
    index += 4;
    
    // Z轴角速度
    uint32_t wz_net = htonl(*reinterpret_cast<uint32_t*>(&wz));
    memcpy(&buffer[index], &wz_net, 4);
    index += 4;
    
    // 计算校验和
    buffer[index++] = calculateChecksum(buffer, index);
    
    length = index;
}

void SerialPort::printHexData(const uint8_t* data, size_t length) {
    std::cout << "HEX: ";
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 8 == 0 && i != length - 1) {
            std::cout << std::endl << "     ";
        }
    }
    std::cout << std::endl;
}

bool SerialPort::validateVelocityCommand(const uint8_t* data, size_t length) {
    // 预期的示例数据 (X:0.2, Y:0, Z:0)
    /*static const uint8_t expected_data[] = {
        0xFE, 0xEF, 0x0D, 0x01,        // 帧头、长度、功能码
        0x3E, 0x4C, 0xCC, 0xCC,        // X速度 0.2
        0x00, 0x00, 0x00, 0x00,        // Y速度 0.0
        0x00, 0x00, 0x00, 0x00,        // Z角速度 0.0
        0x10                            // 校验和
    };*/
    
    if (length != 17) {
        std::cerr << "Validation failed: Data length mismatch (expected 17, got " 
                  << length << ")" << std::endl;
        return false;
    }
    
    /*for (int i = 0; i < 17; i++) {
        if (data[i] != expected_data[i]) {
            std::cerr << "Validation failed: Byte " << i 
                      << " mismatch (expected 0x" << std::hex << (int)expected_data[i]
                      << ", got 0x" << std::hex << (int)data[i] << ")" << std::dec << std::endl;
            return false;
        }
    }*/
    
    // 验证校验和
    uint8_t calculated_checksum = calculateChecksum(data, 16);
    if (data[16] != calculated_checksum) {
        std::cerr << "Validation failed: Checksum mismatch (expected 0x" 
                  << std::hex << (int)calculated_checksum << ", got 0x" 
                  << std::hex << (int)data[16] << ")" << std::dec << std::endl;
        return false;
    }
    
    return true;
}

// ==================== 实用函数 ====================
float SerialPort::toFloat(const uint8_t* bytes) {
    uint32_t int_value = 0;
    int_value = (int_value << 8) | bytes[0];
    int_value = (int_value << 8) | bytes[1];
    int_value = (int_value << 8) | bytes[2];
    int_value = (int_value << 8) | bytes[3];
    
    // 网络字节序转主机字节序
    int_value = ntohl(int_value);
    return *reinterpret_cast<float*>(&int_value);
}

void SerialPort::fromFloat(float value, uint8_t* bytes) {
    uint32_t int_value = *reinterpret_cast<uint32_t*>(&value);
    int_value = htonl(int_value);
    
    bytes[0] = (int_value >> 24) & 0xFF;
    bytes[1] = (int_value >> 16) & 0xFF;
    bytes[2] = (int_value >> 8) & 0xFF;
    bytes[3] = int_value & 0xFF;
}

// ==================== 私有辅助函数 ====================
bool SerialPort::configureSerialPort() {
    struct termios options;
    if (tcgetattr(serial_fd, &options) != 0) {
        std::cerr << "Error: Failed to get serial port attributes (" 
                  << strerror(errno) << ")" << std::endl;
        return false;
    }
    
    // 设置波特率
    speed_t speed;
    switch (baud_rate) {
        case 9600:    speed = B9600; break;
        case 19200:   speed = B19200; break;
        case 38400:   speed = B38400; break;
        case 57600:   speed = B57600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        case 921600:  speed = B921600; break;
        default:
            std::cerr << "Error: Unsupported baud rate: " << baud_rate << std::endl;
            return false;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // 数据位、停止位、奇偶校验
    options.c_cflag &= ~PARENB;        // 无奇偶校验
    options.c_cflag &= ~CSTOPB;        // 1位停止位
    options.c_cflag &= ~CSIZE;         // 清除数据位设置
    options.c_cflag |= CS8;            // 8位数据位
    
    // 启用接收器，忽略调制解调器状态线
    options.c_cflag |= (CLOCAL | CREAD);
    
    // 关闭硬件流控
    options.c_cflag &= ~CRTSCTS;
    
    // 原始输入模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // 关闭软件流控和特殊字符处理
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    
    // 原始输出模式
    options.c_oflag &= ~OPOST;
    
    // 设置超时
    options.c_cc[VMIN] = 0;      // 读取的最小字节数
    options.c_cc[VTIME] = 1;     // 超时时间 (0.1秒单位)
    
    // 应用设置
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
        std::cerr << "Error: Failed to set serial port attributes (" 
                  << strerror(errno) << ")" << std::endl;
        return false;
    }
    
    return true;
}

// 静态成员函数定义
uint8_t SerialPort::calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}
