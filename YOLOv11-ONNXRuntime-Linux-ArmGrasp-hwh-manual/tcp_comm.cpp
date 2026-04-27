#include "tcp_comm.h"


// 构造函数实现
TCPClient::TCPClient(const std::string& ip, int port) 
    : server_ip(ip), server_port(port), sock_fd(-1) {
        
    int maxretries = 3;
    int retries = 0;
    connection_flag = false;
    while(retries < maxretries) {
		if (connect()) {
			std::cout << "Connected successfully!" << std::endl;
			connection_flag = true;
			break;
		} else {
		    retries++;
			std::cerr << "Connection failed, retrying in 1 seconds..." << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
    }
    if(retries == maxretries){
    	std::cerr << "Can not connection!!!" << std::endl;
    }
    
}

//析构函数实现
TCPClient::~TCPClient(){
	std::cout << "Waiting 3 seconds before closing..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    close();
}

// 连接服务器实现
bool TCPClient::connect() {
    // 创建socket
    if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return false;
    }
    
    // 配置服务器地址
    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(server_port);
    
    // 转换IP地址
    if (inet_pton(AF_INET, server_ip.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/Address not supported" << std::endl;
        return false;
    }
    
    // 连接服务器
    if (::connect(sock_fd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return false;
    }
    
    std::cout << "Connected to server: " << server_ip << ":" << server_port << std::endl;
    return true;
}

// 发送消息实现
bool TCPClient::sendMsg(const std::string& message, bool ensure_sent) {
    // 发送消息
    int bytes_sent = send(sock_fd, message.c_str(), message.length(), 0);

    if (bytes_sent < 0) {
        std::cerr << "Send error" << std::endl;
        return false;
    }

    std::cout << "Sent: " << message << std::endl;

    // 如果需要确保完整发送
    if (ensure_sent && (bytes_sent != static_cast<int>(message.length()))) {
        std::cerr << "Message not fully sent" << std::endl;
        return false;
    }

    return true;
}

bool TCPClient::sendPose0(const cv::Point3f& position, const Eigen::Vector3f& euler_angles) {
    float x = position.x;
    float y = position.y;
    float z = position.z;
    float alpha = euler_angles[0];  // 绕X轴旋转角度
    float beta = euler_angles[1];   // 绕Y轴旋转角度
    float gamma = euler_angles[2];  // 绕Z轴旋转角度  
                      
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << x << "," << y << "," << z << ","
        << alpha << "," << beta << "," << gamma;
    
    std::string pose_str = oss.str();
    
    int maxRetries = 3;
    int retries = 0;
    while (retries < maxRetries) {
        ssize_t bytesSent = send(sock_fd, pose_str.c_str(), pose_str.length(), 0);
        if (bytesSent == static_cast<ssize_t>(pose_str.length())) {
            std::cout << "Successfully Sent pose: " << pose_str << std::endl;
            return true; // 发送成功
        } else if (bytesSent == -1) {
            std::cerr << "发送错误，尝试 " << retries + 1 << "/" << maxRetries << std::endl;
            retries++;
            sleep(1); // 等待一段时间后重试
        } else {
            // 部分发送，处理部分发送的情况
            std::cerr << "部分发送，尝试继续发送剩余部分" << std::endl;
        }
    }
    return false; // 重试次数用尽
}

// 发送位姿数据实现
bool TCPClient::sendPose(double x, double y, double z, 
                        double alpha, double beta, double gamma) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << x << "," << y << "," << z << ","
        << alpha << "," << beta << "," << gamma;
    
    std::string pose_str = oss.str();
    int bytes_sent = send(sock_fd, pose_str.c_str(), pose_str.size(), 0);
    
    if (bytes_sent < 0) {
        std::cerr << "Failed to send pose message" << std::endl;
        return false;
    }
    
    std::cout << "Sent pose: " << pose_str << std::endl;
    return true;
}

// 等待特定消息实现
bool TCPClient::waitForMsg(const std::string& expected_msg, int timeout_sec) {
    // 设置超时
    timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 缓冲区大小根据需求调整
    const int buffer_size = 256;
    char buffer[buffer_size] = {0};

    int bytes_read = recv(sock_fd, buffer, buffer_size - 1, 0);

    if (bytes_read < 0) {
        std::cerr << "Receive timeout or error" << std::endl;
        return false;
    }

    buffer[bytes_read] = '\0';
    std::cout << "Received: " << buffer << std::endl;

    // 去除换行符等空白字符
    std::string received_msg(buffer);
    received_msg.erase(received_msg.find_last_not_of(" \n\r\t") + 1);

    return (received_msg == expected_msg);
}

// 等待GO消息实现
bool TCPClient::waitForGO(int timeout_sec) {
    // 设置超时
    timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    char buffer[16] = {0};
    int bytes_read = recv(sock_fd, buffer, sizeof(buffer) - 1, 0);
    
    if (bytes_read < 0) {
        std::cerr << "Receive timeout or error" << std::endl;
        return false;
    }
    
    buffer[bytes_read] = '\0';
    std::cout << "Received: " << buffer << std::endl;
    
    return (strcmp(buffer, "GO") == 0);
}

// 关闭连接实现
void TCPClient::close() {
    if (sock_fd >= 0) {
        ::close(sock_fd);
        sock_fd = -1;
        std::cout << "Connection closed" << std::endl;
    }
}

// 测试通信流程实现
void TCPClient::testCommunication(const cv::Point3f& position, const Eigen::Vector3f& euler_angles) {
    // 从输入参数获取位置和欧拉角
    float x = position.x;
    float y = position.y;
    float z = position.z;
    float alpha = euler_angles[0];  // 绕X轴旋转角度
    float beta = euler_angles[1];   // 绕Y轴旋转角度
    float gamma = euler_angles[2];  // 绕Z轴旋转角度
    
    /*double x = -0.201956, y = 0.624942, z = 0.385797;
    double alpha = -117.642342/180*3.1415926, beta = 81.742828/180*3.1415926, gamma = -17.948023/180*3.1415926;*/

    // 连接重试循环
    /*while(true) {
        if (connect()) {
            std::cout << "Connected successfully!" << std::endl;
            break;
        } else {
            std::cerr << "Connection failed, retrying in 1 seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }*/
    
    // 发送重试循环
    /*while(true) {
        if (sendPose(x, y, z, alpha, beta, gamma)) {
	    std::cout << "Pose sent successfully!" << std::endl;
	    break;
        } else {
	    std::cerr << "Failed to send pose, retrying in 1 second..." << std::endl;
	    std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }*/
    
    sendPose0(position,euler_angles);
    
    // 接收重试循环
    /*while(true) {
        if (waitForMsg("Got")) {
            std::cout << "Successfully received Got signal!" << std::endl;
            break;
        } else {
            std::cerr << "Failed to receive OK, retrying in 1 second..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            // 发送重试循环
	    while(true) {
		if (sendPose(x, y, z, alpha, beta, gamma)) {
		    std::cout << "Pose sent successfully!" << std::endl;
		    break;
		} else {
		    std::cerr << "Failed to send pose, retrying in 1 second..." << std::endl;
		    std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	    }
        }
    }*/
    
    
    
    // 接收重试循环
    while(true) {
        if (waitForMsg("Finish")) {
            std::cout << "Successfully received Finish!" << std::endl;
            break;
        } else {
            std::cerr << "Failed to receive Finish, Waiting..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
        }
    }

    // 等待3秒模拟处理时间
    std::cout << "Waiting 3 seconds before closing..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    close();
}
