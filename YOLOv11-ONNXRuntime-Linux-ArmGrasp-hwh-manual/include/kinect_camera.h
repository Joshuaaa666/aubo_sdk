#pragma once

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <thread>  
#include <chrono>  

class KinectCamera {
public:
    // 构造函数（可配置深度范围）
    KinectCamera(uint16_t min_depth = 50, uint16_t max_depth = 2000);
    ~KinectCamera();
    
    // 初始化并启动相机
    bool initialize();
    
    // 获取下一帧数据
    /**
    * @brief 获取下一帧数据
    * 
    * @param rgb 输出的RGB图像 (1920x1080)
    * @param depth_color 原始深度图的可视化 (640x576)
    * @param aligned_depth 对齐到彩色图的深度数据 (1920x1080)
    * @param aligned_depth_color 对齐到彩色图的深度可视化 (自动调整为1920x1080)
    * @return true 成功获取帧
    * @return false 获取失败
    */
    bool getNextFrame(cv::Mat& rgb, 
                     cv::Mat& depth_color,
                     cv::Mat* aligned_depth = nullptr, 
                     cv::Mat* aligned_depth_color = nullptr);
    
    // 关闭相机
    void shutdown();
    
    /**
     * @brief 将2D像素坐标转换为彩色相机坐标系中的3D点
     * 
     * @param pixel 2D像素坐标 (x, y)
     * @param depth 深度值（毫米）
     * @param valid 输出参数，指示转换是否成功
     * @return cv::Point3f 3D点坐标 (X, Y, Z) 单位：毫米
     */
    cv::Point3f pixelToPoint3D(const cv::Point2f& pixel, uint16_t depth, int* valid = nullptr);
    
    /**
     * @brief 将多个2D像素坐标批量转换为彩色相机坐标系中的3D点
     * 
     * @param pixels 2D像素坐标向量
     * @param depths 对应的深度值向量（毫米）
     * @return std::vector<cv::Point3f> 3D点坐标向量
     */
    std::vector<cv::Point3f> pixelsToPoints3D(const std::vector<cv::Point2f>& pixels, 
                                             const std::vector<uint16_t>& depths);
                                             
    /**
     * @brief 将OpenCV深度图转换为PCL点云
     * 
     * @param depth_mat 输入的深度图像 (CV_16UC1, 单位毫米)
     * @param cloud 输出的PCL点云
     * @param rgb_mat 可选的RGB图像 (CV_8UC3)
     * @return true 转换成功
     * @return false 转换失败
     */
    bool depthImageToPointCloud(const cv::Mat& depth_mat,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                              const cv::Mat& rgb_mat = cv::Mat()
                              );

    /**
     * @brief 可视化PCL点云
     * 
     * @param cloud 要可视化的点云
     * @param window_name 窗口名称
     * @param background_color 背景颜色 (默认黑色)
     */
    /*void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                            const std::string& window_name = "Point Cloud Viewer",
                            const cv::Scalar& background_color = cv::Scalar(0, 0, 0));*/
    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                           const std::string& window_name,
                                           const cv::Scalar& background_color,
                                           const cv::Point3f& position,         
                                           const Eigen::Vector3f& euler_angles,    
                                           float axis_length,                    
                                           const std::string& frame_id /* = "pose_frame" */);
    /**
    * @brief 对输入点云进行欧式聚类，并返回其中点数最多的簇
    * @param cloud 输入点云 (Ptr)
    * @param cluster_tolerance 聚类时的距离阈值（默认 0.02 m）
    * @param min_cluster_size 最小簇大小（默认 100 点）
    * @param max_cluster_size 最大簇大小（默认 25000 点）
    * @return 包含最大簇的点云 Ptr；如果没有有效簇，则返回空 Ptr
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractLargestEuclideanCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    double cluster_tolerance = 0.01,
    int min_cluster_size   = 100,
    int max_cluster_size   = 25000);                                       
   
private:
    // 将原始深度图转换为彩色可视化图像
    cv::Mat convertDepthToColor(const cv::Mat& depth_raw);
    
    // 校准设备
    k4a::device device;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_device_configuration_t config;
    
    uint16_t min_depth_;
    uint16_t max_depth_;
    float depth_scale_;
    bool initialized = false;
};
