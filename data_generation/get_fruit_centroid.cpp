#include "kinect_camera.h"
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <limits>
#include <cstdlib>  
#include <fstream>
#include <sstream>

// 计算点云质心的核心函数
bool computeCloudCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3d& center_cam) {
    if (!cloud || cloud->empty()) {
        return false;
    }

    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*cloud, centroid4);
    center_cam = centroid4.head<3>().cast<double>();
    return true;
}

// 手眼标定矩阵：工具坐标系到相机坐标系的变换矩阵
// 注意：该平移量看起来是毫米，这里会统一转换为米以匹配点云单位。
Eigen::Matrix4d buildToolToCamera() {
    Eigen::Matrix4d tool_T_cam;
    tool_T_cam <<
        0.13551828522285903,  0.99077427969707599,  0.0010588016795823466, -135.91254527050771,
       -0.99077376072521717,  0.13551964081471257, -0.0013349201705961042,  -51.733981386034912,
       -0.0014666929937864665,-0.0008681268295133869, 0.99999854846251723,  -90.646138164038007,
        0.0,                  0.0,                  0.0,                    1.0;

    // mm -> m
    tool_T_cam(0, 3) /= 1000.0;
    tool_T_cam(1, 3) /= 1000.0;
    tool_T_cam(2, 3) /= 1000.0;
    return tool_T_cam;
}

// 罗德里格斯旋转向量(rx, ry, rz) -> 旋转矩阵。
Eigen::Matrix3d rotvecToMatrix(const Eigen::Vector3d& rotvec) {
    const double theta = rotvec.norm();
    if (theta < 1e-12) {
        return Eigen::Matrix3d::Identity();
    }
    const Eigen::Vector3d axis = rotvec / theta;
    return Eigen::AngleAxisd(theta, axis).toRotationMatrix();
}

Eigen::Matrix4d poseToMatrix(const std::vector<double>& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (pose.size() < 6) {
        return T;
    }
    T.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    T.block<3, 3>(0, 0) = rotvecToMatrix(Eigen::Vector3d(pose[3], pose[4], pose[5]));
    return T;
}

// 将相机坐标系下的点转换到机械臂基座坐标系
Eigen::Vector3d transformCamPointToBase(const Eigen::Vector3d& point_cam,
                                        const Eigen::Matrix4d& base_T_tool,
                                        const Eigen::Matrix4d& tool_T_cam) {
    Eigen::Vector4d p_cam(point_cam.x(), point_cam.y(), point_cam.z(), 1.0);
    Eigen::Vector4d p_base = base_T_tool * tool_T_cam * p_cam;
    return p_base.head<3>();
}

// 从Python脚本获取真实的机械臂位姿
bool getBaseToToolFromRobotReal(Eigen::Matrix4d& base_T_tool) {
    // 调用Python脚本来获取机械臂的真实位姿
    std::string command = "python2 /home/lingao/Desktop/Python_sdk_linux/python_test/get_data.py 2>/dev/null";
    
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "无法执行Python脚本获取机械臂位置" << std::endl;
        return false;
    }
    
    char buffer[1024];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
        result += buffer;
    }
    pclose(pipe);
    std::cout << "DEBUG: Python脚本输出内容:\n" << result << std::endl;
    // 检查是否成功获取数据
    if (result.find("笛卡尔坐标(米)") == std::string::npos) {
        std::cerr << "Python脚本执行失败或未返回有效数据" << std::endl;
        std::cerr << "返回内容: " << result << std::endl;
        return false;
    }
    
    // 解析输出，提取位置和姿态信息
    std::istringstream iss(result);
    std::string line;
    double x = 0, y = 0, z = 0;
    double rx = 0, ry = 0, rz = 0;
    bool found_pos = false;
    
    while (std::getline(iss, line)) {
        std::cout << "DEBUG: 处理行: " << line << std::endl;
        // 查找笛卡尔坐标的行
        if (line.find("笛卡尔坐标(米)") != std::string::npos) {
            // 解析 X, Y, Z 坐标
            size_t x_pos = line.find("X=");
            size_t y_pos = line.find(", Y=");
            size_t z_pos = line.find(", Z=");
            
            if (x_pos != std::string::npos && y_pos != std::string::npos && z_pos != std::string::npos) {
                x = std::stod(line.substr(x_pos + 2, y_pos - x_pos - 2));
                y = std::stod(line.substr(y_pos + 4, z_pos - y_pos - 4));
                z = std::stod(line.substr(z_pos + 4));
                found_pos = true;
            }
        }
        // 查找欧拉角姿态的行
        else if (line.find("欧拉角姿态(弧度)") != std::string::npos) {
            // 解析 RX, RY, RZ 角度
            size_t rx_pos = line.find("RX=");
            size_t ry_pos = line.find(", RY=");
            size_t rz_pos = line.find(", RZ=");
            
            if (rx_pos != std::string::npos && ry_pos != std::string::npos && rz_pos != std::string::npos) {
                rx = std::stod(line.substr(rx_pos + 3, ry_pos - rx_pos - 3));
                ry = std::stod(line.substr(ry_pos + 5, rz_pos - ry_pos - 5));
                rz = std::stod(line.substr(rz_pos + 5));
            }
        }
    }
    
    if (found_pos) {
        // 构建变换矩阵
        std::vector<double> pose = {x, y, z, rx, ry, rz};
        base_T_tool = poseToMatrix(pose);
        return true;
    } else {
        std::cerr << "无法从Python脚本输出中解析机械臂位姿" << std::endl;
        return false;
    }
}

// 获取机器人当前状态（模拟模式，返回固定姿态）
bool getBaseToToolFromRobotSimulated(Eigen::Matrix4d& base_T_tool) {
    // 模拟一个固定的机械臂姿态
    // 这里使用一个示例姿态：机械臂在初始位置
    std::vector<double> simulated_pose = {1.5895, -0.4545, -2.1783, -1.5664, -1.5630, 0.7595};
    base_T_tool = poseToMatrix(simulated_pose);
    return true;
}

int main() {
    std::unique_ptr<KinectCamera> kinect = std::make_unique<KinectCamera>();
    if (!kinect->initialize()) {
        std::cerr << "Kinect初始化失败！" << std::endl;
        return -1;
    }

    // 获取手眼标定矩阵
    const Eigen::Matrix4d tool_T_cam = buildToolToCamera();

    cv::Mat frame;
    cv::Mat raw_frame;
    cv::Mat aligned_depth;
    cv::Mat aligned_depth_color;

    cv::Rect resultBox;
    bool has_roi = false;

    std::cout << "=== 水果质心获取工具（含坐标变换） ===" << std::endl;
    std::cout << "按键说明: r=选择目标框, s=获取水果质心并转换坐标, ESC=退出" << std::endl;
    
    // 添加时间戳的日志文件
    time_t now = time(0);
    char timestamp[100];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", localtime(&now));
    std::string log_filename = "/tmp/fruit_detection_log_" + std::string(timestamp) + ".txt";
    std::ofstream log_file(log_filename);
    if(log_file.is_open()) {
        log_file << "=== 水果质心检测日志 ===" << std::endl;
        log_file << "开始时间: " << ctime(&now);
        log_file << "=========================================" << std::endl;
    }


    while (true) {
        // 获取Kinect帧
        if (!kinect->getNextFrame(frame, aligned_depth_color, &aligned_depth, &aligned_depth_color)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (frame.empty()) {
            continue;
        }

        raw_frame = frame.clone();

        // 显示ROI框（如果已选择）
        if (has_roi) {
            cv::rectangle(frame, resultBox, cv::Scalar(0, 255, 0), 2);
            if (!aligned_depth_color.empty()) {
                cv::rectangle(aligned_depth_color, resultBox, cv::Scalar(0, 255, 0), 2);
            }
        }

        cv::imshow("RGB", frame);
        cv::imshow("Aligned Depth", aligned_depth_color);

        const int key = cv::waitKey(20) & 0xFF;

        if (key == 'r' || key == 'R') {
            // 选择ROI区域
            cv::Rect selected = cv::selectROI("RGB", raw_frame, false, false);
            if (selected.width > 0 && selected.height > 0) {
                resultBox = selected;
                has_roi = true;
                std::cout << "ROI选择完成: [" 
                          << resultBox.x << ", " << resultBox.y << ", "
                          << resultBox.width << ", " << resultBox.height << "]" << std::endl;
            } else {
                std::cout << "ROI选择已取消。" << std::endl;
            }
        }

        if (key == 's' || key == 'S') {
            // 获取水果质心
            if (!has_roi) {
                std::cout << "请先按 r 选择目标框。" << std::endl;
                continue;
            }
            if (aligned_depth.empty()) {
                std::cout << "深度图为空，跳过处理。" << std::endl;
                continue;
            }

            // 生成ROI点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (!kinect->depthImageToPointCloudInROI(aligned_depth, resultBox, roi_cloud, raw_frame)) {
                std::cout << "ROI点云生成失败。" << std::endl;
                continue;
            }

            // 提取最大的欧几里得聚类（去除噪声）
            auto target_cloud = kinect->extractLargestEuclideanCluster(roi_cloud, 0.01, 80, 300000);
            if (!target_cloud || target_cloud->empty()) {
                target_cloud = roi_cloud;
            }

            // 计算质心（相机坐标系）
            Eigen::Vector3d center_cam;
            if (!computeCloudCentroid(target_cloud, center_cam)) {
                std::cout << "无法计算点云质心。" << std::endl;
                continue;
            }

            // 获取机械臂当前姿态（基座到工具的变换矩阵）
            Eigen::Matrix4d base_T_tool = Eigen::Matrix4d::Identity();
            bool has_base_pose = getBaseToToolFromRobotReal(base_T_tool);  // 使用真实的机械臂姿态
            
            // 如果无法获取真实姿态，使用模拟姿态
            if (!has_base_pose) {
                std::cout << "无法获取真实机械臂姿态，使用模拟姿态(固定初始姿态)" << std::endl;
                has_base_pose = getBaseToToolFromRobotSimulated(base_T_tool);
            }

            // 转换到机械臂基座坐标系
            Eigen::Vector3d center_base = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            if (has_base_pose) {
                center_base = transformCamPointToBase(center_cam, base_T_tool, tool_T_cam);
            }

            // 输出结果
            std::cout << "\n=== 水果质心坐标 ===" << std::endl;
            std::cout << "相机坐标系（单位：米）:" << std::endl;
            std::cout << "  X: " << center_cam.x() << " m" << std::endl;
            std::cout << "  Y: " << center_cam.y() << " m" << std::endl;
            std::cout << "  Z: " << center_cam.z() << " m" << std::endl;
            
            if (has_base_pose) {
                std::cout << "机械臂基座坐标系（单位：米）:" << std::endl;
                std::cout << "  X: " << center_base.x() << " m" << std::endl;
                std::cout << "  Y: " << center_base.y() << " m" << std::endl;
                std::cout << "  Z: " << center_base.z() << " m" << std::endl;
                
                

                // 调用Python 2脚本控制机械臂移动到目标位置
                char command[500];
                sprintf(command, "python2 /home/lingao/Desktop/Python_sdk_linux/python_test/move_to_position.py %.6f %.6f %.6f", 
                        center_base.x(), center_base.y(), center_base.z());
                std::cout << "执行命令: " << command << std::endl;
                
                int ret = std::system(command);
                if (ret == 0) {
                    std::cout << "机械臂移动命令发送成功！" << std::endl;
                } else {
                    std::cout << "机械臂移动命令发送失败！返回值: " << ret << std::endl;
                    std::cout << "可能的原因是目标位置超出机械臂工作空间" << std::endl;
                }
            } else {
                std::cout << "机械臂基座坐标系：未获取到机械臂姿态，无法转换" << std::endl;
            }
            std::cout << "====================\n" << std::endl;
        }
        if (key == 't' || key == 'T') {
        // 测试简单的目标位置
        Eigen::Vector3d test_base(0.15, -0.25, 0.45);
        std::cout << "测试移动至已知安全位置: " << test_base.transpose() << std::endl;
        
        char command[500];
        sprintf(command, "python2 /home/lingao/Desktop/Python_sdk_linux/python_test/move_to_position.py %.6f %.6f %.6f", 
                test_base.x(), test_base.y(), test_base.z());
        system(command);
    }
        if (key == 27) {
            break;
        }
    }
    if(log_file.is_open()) {
        log_file << "结束时间: " << ctime(&now);
        log_file << "程序结束" << std::endl;
        log_file.close();
        std::cout << "日志已保存到: " << log_filename << std::endl;
    }
    
    kinect->shutdown();
    cv::destroyAllWindows();
    return 0;
}