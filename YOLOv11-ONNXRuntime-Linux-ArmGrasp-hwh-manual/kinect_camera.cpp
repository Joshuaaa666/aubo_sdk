#include "kinect_camera.h"
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

KinectCamera::KinectCamera(uint16_t min_depth, uint16_t max_depth)
    : min_depth_(min_depth), max_depth_(max_depth) {
    depth_scale_ = 255.0f / (max_depth_ - min_depth_);
}

KinectCamera::~KinectCamera() {
    shutdown();
}

bool KinectCamera::initialize() {
    if (k4a::device::get_installed_count() == 0) {
        std::cerr << "No Azure Kinect devices detected!" << std::endl;
        return false;
    }

    try {
        device = k4a::device::open(K4A_DEVICE_DEFAULT);
        
        // 配置相机参数
        config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        config.camera_fps = K4A_FRAMES_PER_SECOND_15;
        config.synchronized_images_only = true;
        
        device.start_cameras(&config);
        
        // 获取校准数据并创建变换对象
        calibration = device.get_calibration(config.depth_mode, config.color_resolution);      
        transformation = k4a::transformation(calibration);
        
        initialized = true;
        return true;
    } catch (const k4a::error& e) {
        std::cerr << "Kinect initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool KinectCamera::getNextFrame(cv::Mat& rgb, 
                               cv::Mat& depth_color,
                               cv::Mat* aligned_depth,
                               cv::Mat* aligned_depth_color) {
    if (!initialized) return false;

    k4a::capture capture;
    if (device.get_capture(&capture, std::chrono::milliseconds(100))) {
        // 处理RGB图像
        k4a::image colorImage = capture.get_color_image();
        if (colorImage != nullptr) {
            cv::Mat cvColor(colorImage.get_height_pixels(), colorImage.get_width_pixels(),
                            CV_8UC4, (void*)colorImage.get_buffer());
            cv::cvtColor(cvColor, rgb, cv::COLOR_BGRA2BGR);
            colorImage.reset();
        } else {
            return false;
        }

        // 处理深度图像
        k4a::image depthImage = capture.get_depth_image();
        if (depthImage != nullptr) {
            cv::Mat depthRaw(depthImage.get_height_pixels(), depthImage.get_width_pixels(),
                             CV_16U, (void*)depthImage.get_buffer());
            depth_color = convertDepthToColor(depthRaw);
            
            // 深度图对齐到彩色图
            if (aligned_depth || aligned_depth_color) {
                try {
                    // 将深度图转换到彩色图坐标系
                    k4a::image transformed_depth = transformation.depth_image_to_color_camera(depthImage);
                    
                    // 获取对齐后的深度数据
                    if (aligned_depth) {
                        *aligned_depth = cv::Mat(transformed_depth.get_height_pixels(),
                                               transformed_depth.get_width_pixels(),
                                               CV_16U,
                                               (void*)transformed_depth.get_buffer()).clone();
                    }
                    
                    // 获取对齐后的深度可视化
                    if (aligned_depth_color) {
                        cv::Mat aligned_depth_raw = cv::Mat(transformed_depth.get_height_pixels(),
                                                          transformed_depth.get_width_pixels(),
                                                          CV_16U,
                                                          (void*)transformed_depth.get_buffer());
                        *aligned_depth_color = convertDepthToColor(aligned_depth_raw);
                        cv::resize(*aligned_depth_color, *aligned_depth_color, depth_color.size());
                    }
                    
                    transformed_depth.reset();
                } catch (const k4a::error& e) {
                    std::cerr << "Depth transformation failed: " << e.what() << std::endl;
                }
            }
            
            depthImage.reset();
            return true;
        }
    }
    return false;
}

cv::Mat KinectCamera::convertDepthToColor(const cv::Mat& depth_raw) {
    cv::Mat depth_normalized(depth_raw.size(), CV_8U);
    
    for (int y = 0; y < depth_raw.rows; y++) {
        const uint16_t* depth_row = depth_raw.ptr<uint16_t>(y);
        uint8_t* norm_row = depth_normalized.ptr<uint8_t>(y);
        
        for (int x = 0; x < depth_raw.cols; x++) {
            uint16_t depth_value = depth_row[x];
            
            if (depth_value == 0) {
                norm_row[x] = 0;
            } else if (depth_value < min_depth_) {
                norm_row[x] = 0;
            } else if (depth_value > max_depth_) {
                norm_row[x] = 255;
            } else {
                norm_row[x] = static_cast<uint8_t>((depth_value - min_depth_) * depth_scale_);
            }
        }
    }
    
    cv::Mat depth_color;
    cv::applyColorMap(depth_normalized, depth_color, cv::COLORMAP_JET);
    
    // 将无效点设为黑色
    cv::Mat mask = (depth_raw == 0);
    depth_color.setTo(cv::Scalar(0, 0, 0), mask);
    
    return depth_color;
}

void KinectCamera::shutdown() {
    if (initialized) {
        transformation.destroy();
        device.stop_cameras();
        device.close();
        initialized = false;
    }
}

cv::Point3f KinectCamera::pixelToPoint3D(const cv::Point2f& pixel, uint16_t depth, int* valid) {
    const int INVALID = 0;
    const int VALID = 1;
    
    // 初始化有效性标志
    if (valid) *valid = INVALID;
    
    // 检查初始化状态
    if (!initialized) {
        std::cerr << "Error: Camera not initialized" << std::endl;
        return cv::Point3f(0, 0, 0);
    }

    // 检查深度值有效性
    if (depth < 100 || depth > 4000) {  // 根据你的设备调整范围
        std::cerr << "Invalid depth: " << depth << std::endl;
        return cv::Point3f(0, 0, 0);
    }
     
    // 准备数据结构
    k4a_float2_t point_2d = {pixel.x, pixel.y};
    k4a_float3_t point_3d = {0};
    
    // 执行转换
    int conversion_valid = INVALID;
    
    k4a_calibration_2d_to_3d(
        &calibration,
        &point_2d,
        static_cast<float>(depth),
        K4A_CALIBRATION_TYPE_COLOR,
        K4A_CALIBRATION_TYPE_COLOR, 
        &point_3d,
        &conversion_valid
    );

    // 设置有效性标志
    if (valid) *valid = conversion_valid;

    // 检查转换结果
    if (!conversion_valid) {
        std::cerr << "Conversion failed for point ("
                  << pixel.x << ", " << pixel.y << ") depth: " << depth << std::endl;
    }

    return cv::Point3f(point_3d.xyz.x, point_3d.xyz.y, point_3d.xyz.z);
}

std::vector<cv::Point3f> KinectCamera::pixelsToPoints3D(
    const std::vector<cv::Point2f>& pixels, 
    const std::vector<uint16_t>& depths) {
    
    std::vector<cv::Point3f> points;
    
    if (pixels.size() != depths.size() || !initialized) {
        return points;
    }
    
    for (size_t i = 0; i < pixels.size(); i++) {
        int valid = 0;
        cv::Point3f point = pixelToPoint3D(pixels[i], depths[i], &valid);
        if (valid) {
            points.push_back(point);
        }
    }
    
    return points;
}

bool KinectCamera::depthImageToPointCloud(const cv::Mat& depth_mat,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                          const cv::Mat& rgb_mat) {
    if (!initialized || depth_mat.empty() || depth_mat.type() != CV_16UC1) {
        std::cerr << "Invalid depth image format! Expected CV_16UC1." << std::endl;
        return false;
    }

    // 初始化点云对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    raw_cloud->is_dense = false;

    // 相机内参
    const k4a_calibration_intrinsic_parameters_t* intrinsics =
        &calibration.color_camera_calibration.intrinsics.parameters;

    const double fx = intrinsics->param.fx;
    const double fy = intrinsics->param.fy;
    const double cx = intrinsics->param.cx;
    const double cy = intrinsics->param.cy;

    for (int v = 0; v < depth_mat.rows; ++v) {
        for (int u = 0; u < depth_mat.cols; ++u) {
            uint16_t depth_value = depth_mat.at<uint16_t>(v, u);
            if (depth_value == 0) continue;  // 无效深度

            double z = depth_value / 1000.0;
            if (z <= 0 || std::isnan(z)) continue;

            double x = (u - cx) * z / fx;
            double y = (v - cy) * z / fy;

            if (std::isnan(x) || std::isnan(y)) continue;

            pcl::PointXYZRGB pt;
            pt.x = static_cast<float>(x);
            pt.y = static_cast<float>(y);
            pt.z = static_cast<float>(z);

            if (!rgb_mat.empty() && rgb_mat.type() == CV_8UC3) {
                const cv::Vec3b& color = rgb_mat.at<cv::Vec3b>(v, u);
                pt.r = color[2];
                pt.g = color[1];
                pt.b = color[0];
            } else {
                pt.r = pt.g = pt.b = 255;
            }

            raw_cloud->points.emplace_back(std::move(pt));
        }
    }

    // 设置点云组织结构
    raw_cloud->width = static_cast<uint32_t>(raw_cloud->points.size());
    raw_cloud->height = 1;
    raw_cloud->is_dense = false;

    // 移除 NaN（防止某些浮点异常未被捕捉）
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*raw_cloud, *cloud, valid_indices);

    if (cloud->empty()) {
        std::cerr << "All points filtered out (no valid points remaining)." << std::endl;
        return false;
    }

    return true;
}


void KinectCamera::visualizePointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& window_name,
    const cv::Scalar& background_color,
    const cv::Point3f& position,
    const Eigen::Vector3f& euler_angles,
    float axis_length,
    const std::string& frame_id) 
{
    // 创建PCL可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
    viewer->setBackgroundColor(
        background_color[0] / 255.0,
        background_color[1] / 255.0,
        background_color[2] / 255.0
    );
    
    // 添加点云
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"
    );
    
    // 添加全局坐标系
    viewer->addCoordinateSystem(0.1, "global_frame");
    
    // 绘制点及其姿态
    if (axis_length > 0) {
        // 1. 在指定位置绘制标记点
        viewer->addSphere(
            pcl::PointXYZ(position.x, position.y, position.z), 
            0.005, 1.0, 1.0, 0.0, frame_id + "_point"
        );
        
        // 2. 将欧拉角转换为旋转矩阵 (Yaw-Pitch-Roll顺序)
        Eigen::Matrix3f rotation_matrix = 
            (Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitZ()) *  // Yaw (Z)
             Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) *  // Pitch (Y)
             Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitX())).matrix(); // Roll (X)
        
        // 3. 计算各轴端点
        Eigen::Vector3f pos_vec(position.x, position.y, position.z);
        Eigen::Vector3f x_axis = pos_vec + rotation_matrix * Eigen::Vector3f(axis_length, 0, 0);
        Eigen::Vector3f y_axis = pos_vec + rotation_matrix * Eigen::Vector3f(0, axis_length, 0);
        Eigen::Vector3f z_axis = pos_vec + rotation_matrix * Eigen::Vector3f(0, 0, axis_length);
        
        // 4. 绘制坐标轴
        viewer->addLine(
            pcl::PointXYZ(position.x, position.y, position.z),
            pcl::PointXYZ(x_axis.x(), x_axis.y(), x_axis.z()),
            1.0, 0.0, 0.0, frame_id + "_x_axis"
        );
        viewer->addLine(
            pcl::PointXYZ(position.x, position.y, position.z),
            pcl::PointXYZ(y_axis.x(), y_axis.y(), y_axis.z()),
            0.0, 1.0, 0.0, frame_id + "_y_axis"
        );
        viewer->addLine(
            pcl::PointXYZ(position.x, position.y, position.z),
            pcl::PointXYZ(z_axis.x(), z_axis.y(), z_axis.z()),
            0.0, 0.0, 1.0, frame_id + "_z_axis"
        );
    }
    
    // 主循环
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectCamera::extractLargestEuclideanCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size)
{
    if (!cloud || cloud->empty()) {
        PCL_ERROR("输入点云为空，无法聚类。\n");
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
    }
    std::cout<<"原始点云数量："<<cloud->points.size()<<std::endl;
    
    // 1. 构建 KD 树索引
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    // 2. 欧式聚类提取器
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud); 
    ec.extract(cluster_indices);
    
    if (cluster_indices.empty()) {
        PCL_WARN("未检测到任何簇。\n");
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
    }

    // 3. 找到最大的簇
    std::size_t largest_cluster_idx = 0;
    std::size_t max_size_found = 0;
    for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
        if (cluster_indices[i].indices.size() > max_size_found) {
            largest_cluster_idx = i;
            max_size_found = cluster_indices[i].indices.size();
        }
    }

    // 4. 提取最大簇
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr largest_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointIndices::Ptr largest_indices(new pcl::PointIndices(cluster_indices[largest_cluster_idx]));
    extract.setInputCloud(cloud);  
    extract.setIndices(largest_indices);
    extract.setNegative(false);
    extract.filter(*largest_cloud);
    
    if (largest_cloud && !largest_cloud->empty()) {
	  std::cout << "最大簇点数: " << largest_cloud->size() << std::endl;
    } else {
	  std::cout << "未提取到有效簇。" << std::endl;
    }

    return largest_cloud;
}
