// save_fruit_coords.cpp
#include "kinect_camera.h"
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <limits>
#include <fstream>

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

int main() {
    std::unique_ptr<KinectCamera> kinect = std::make_unique<KinectCamera>();
    if (!kinect->initialize()) {
        std::cerr << "Kinect初始化失败！" << std::endl;
        return -1;
    }

    cv::Mat frame;
    cv::Mat raw_frame;
    cv::Mat aligned_depth;
    cv::Mat aligned_depth_color;

    cv::Rect resultBox;
    bool has_roi = false;

    std::cout << "=== 水果质心获取工具（仅保存相机坐标） ===" << std::endl;
    std::cout << "按键说明: r=选择目标框, s=获取水果质心并保存坐标, ESC=退出" << std::endl;

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

            // 输出结果
            std::cout << "\n=== 水果质心坐标（相机坐标系） ===" << std::endl;
            std::cout << "X: " << center_cam.x() << " m" << std::endl;
            std::cout << "Y: " << center_cam.y() << " m" << std::endl;
            std::cout << "Z: " << center_cam.z() << " m" << std::endl;
            std::cout << "=====================================\n" << std::endl;

            // 将坐标保存到文件，供Python程序读取
            std::ofstream coord_file("/tmp/fruit_coords_camera.txt");
            if (coord_file.is_open()) {
                coord_file << center_cam.x() << " " << center_cam.y() << " " << center_cam.z() << std::endl;
                coord_file.close();
                std::cout << "相机坐标系下的水果质心已保存到 /tmp/fruit_coords_camera.txt" << std::endl;
                
                std::cout << "请运行以下命令进行坐标变换和机械臂控制:" << std::endl;
                std::cout << "python2 /home/lingao/Desktop/Python_sdk_linux/python_test/calculate_fruit_position.py" << std::endl;
            } else {
                std::cout << "无法保存坐标到文件" << std::endl;
            }
        }

        if (key == 27) {
            break;
        }
    }

    kinect->shutdown();
    cv::destroyAllWindows();
    return 0;
}