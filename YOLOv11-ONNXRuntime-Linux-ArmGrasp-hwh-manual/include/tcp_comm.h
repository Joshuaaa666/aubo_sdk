#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <opencv2/core.hpp>
#include <Eigen/Dense>

class TCPClient {
public:
    /**
     * @brief 构造函数，创建TCPClient实例
     * @param ip 服务器IP地址
     * @param port 服务器端口号
     */
    TCPClient(const std::string& ip, int port);
    
    /**
     * @brief 析构函数
    */
    ~TCPClient();
    
    /**
     * @brief 连接到服务器
     * @return 连接成功返回true，失败返回false
     */
     
    bool connection_flag;   // Tcp连接标志
     
    bool connect();
    
    /**
     * @brief 发送消息
     * @param message 要发送的消息字符串
     * @param ensure_sent 是否确保完整发送（默认true）
     * @return 发送成功返回true，失败返回false
     */
    bool sendMsg(const std::string& message, bool ensure_sent = true);
    
    /**
     * @brief 发送位姿数据
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param alpha 绕X轴旋转角度
     * @param beta 绕Y轴旋转角度
     * @param gamma 绕Z轴旋转角度
     * @return 发送成功返回true，失败返回false
     */
    bool sendPose(double x, double y, double z, 
                 double alpha, double beta, double gamma);
                 
    
    bool sendPose0(const cv::Point3f& position, const Eigen::Vector3f& euler_angles);
    
    /**
     * @brief 等待接收特定消息
     * @param expected_msg 期望接收的消息
     * @param timeout_sec 超时时间（秒，默认5秒）
     * @return 接收到期望消息返回true，超时或错误返回false
     */
    bool waitForMsg(const std::string& expected_msg, int timeout_sec = 5);
    
    /**
     * @brief 等待接收"GO"消息
     * @param timeout_sec 超时时间（秒，默认5秒）
     * @return 接收到"GO"返回true，超时或错误返回false
     */
    bool waitForGO(int timeout_sec = 5);
    
    /**
     * @brief 关闭连接
     */
    void close();
    
    /**
     * @brief 测试通信流程
     * 包括连接、发送OK、接收OK、发送位姿、接收GO等步骤
     */
    void testCommunication(const cv::Point3f& position, const Eigen::Vector3f& euler_angles);

private:
    std::string server_ip;  // 服务器IP地址
    int server_port;        // 服务器端口号
    int sock_fd;            // 套接字文件描述符
};

#endif // TCPCLIENT_H
