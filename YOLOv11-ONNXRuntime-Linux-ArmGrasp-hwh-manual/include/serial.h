// serial.h
#ifndef SERIAL_H
#define SERIAL_H

#include <cstdint>
#include <string>
#include <cstddef>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();
    
    // 串口基本操作
    bool open(const std::string& port_name, int baud_rate);
    bool send(const uint8_t* data, size_t length);
    size_t receive(uint8_t* buffer, size_t buffer_size);
    void close();
    bool isOpen() const;
    
    // 速度控制相关功能（静态函数）
    static void createVelocityCommand(float vx, float vy, float wz, uint8_t* buffer, size_t& length);
    static void printHexData(const uint8_t* data, size_t length);
    static bool validateVelocityCommand(const uint8_t* data, size_t length);
    
    // 实用函数（静态函数）
    static float toFloat(const uint8_t* bytes);
    static void fromFloat(float value, uint8_t* bytes);
    
private:
    int serial_fd;
    std::string port_name;
    int baud_rate;
    
    // 私有辅助函数
    bool configureSerialPort();
    
    // 静态辅助函数
    static uint8_t calculateChecksum(const uint8_t* data, size_t length);
    
    // 禁用拷贝构造函数和赋值运算符
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
};

#endif // SERIAL_H
