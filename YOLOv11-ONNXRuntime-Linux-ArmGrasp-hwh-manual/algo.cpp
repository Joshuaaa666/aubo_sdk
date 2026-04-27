#include "algo.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

Algo::Algo()
{
	model_path_ = "/home/lingao/yolo_cpp_rewrite/YOLOv11-ONNXRuntime-Linux-ArmGrasp-hwh/a3.onnx";
	logid_ = "yolo_inference";
	provider_ = "CPU"; // "CPU" or "CUDA"

	inferencer_.reset(new YoloInferencer());
}

Algo::~Algo()
{
}

void Algo::init() {
	inferencer_->init(model_path_, logid_, provider_);
}

bool Algo::inferResult(cv::Mat& image, cv::Rect& highestBox) {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Detection> detections = inferencer_->infer(image, 0.70, 0.5);

    std::cout << "Detection: Count: " << detections.size() << std::endl;

    // 如果没有检测到目标，返回false
    if (detections.empty()) {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "Time taken: " << elapsed.count() << " ms" << std::endl;

        //cv::resize(image, image, cv::Size(640, 480));
        //cv::imshow("output", image);
        cv::waitKey(10);
        return false;
    }

    // 找到置信度最高的检测
    auto maxDetection = std::max_element(detections.begin(), detections.end(),
        [](const Detection& a, const Detection& b) {
            return a.confidence < b.confidence;
        });

    // 只处理最高置信度的检测
    const Detection& detection = *maxDetection;

    // 绘制矩形框
    cv::rectangle(image, detection.box, cv::Scalar(255, 0, 0), 5);

    // 生成标签文本
    std::string label = cv::format("%d: %.2f", detection.class_id, detection.confidence);

    // 计算文本位置
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    int text_x = std::max(detection.box.x, 0);
    int text_y = std::max(detection.box.y - textSize.height - 5, 0);

    // 绘制文本
    cv::putText(image, label,
                cv::Point(text_x, text_y + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX,
                3,
                cv::Scalar(255, 0, 0),
                5);

    // 输出检测信息
    /*std::cout << "Highest Confidence Detection: Class=" << detection.class_id
              << ", Confidence=" << detection.confidence
              << ", x=" << detection.box.x
              << ", y=" << detection.box.y
              << ", width=" << detection.box.width
              << ", height=" << detection.box.height
              << std::endl;*/

    // 设置返回的box
    highestBox = detection.box;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Time taken: " << elapsed.count() << " ms" << std::endl;

    //cv::resize(image, image, cv::Size(640, 480));
    //cv::imshow("output", image);
    //cv::waitKey(10);
    return true;
}


void Algo::setModePath(const std::string mode_str)
{
	model_path_ = mode_str;
}

void Algo::setStride(const int stride)
{
	inferencer_->setStride(stride);
}

//获取小番茄的质心
cv::Point Algo::calculateCentroid(cv::Mat& image, const cv::Rect& box, cv::Mat& mask) {
    // 确保输入图像有效
    if (image.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return cv::Point(-1, -1);
    }

    // 转换为HSV颜色空间（更适合颜色分割）
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 定义红色范围的HSV阈值（两个范围：0-10和170-180）
    cv::Mat mask1, mask2, red_mask;
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, red_mask);

    // 创建ROI掩码，只保留检测框内的区域
    cv::Mat roi_mask = cv::Mat::zeros(red_mask.size(), CV_8UC1);
    cv::rectangle(roi_mask, box, cv::Scalar(255), cv::FILLED);
    cv::bitwise_and(red_mask, roi_mask, red_mask);

    // 形态学操作去除噪点
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    
    // 先闭运算填充内部空洞
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
    
    // 再开运算去除外部噪点
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);

    /*// 查找最大连通区域（避免多个红色区域干扰）
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 如果没有找到轮廓，返回无效点
    if (contours.empty()) {
        return cv::Point(-1, -1);
    }
    
    // 找到面积最大的轮廓
    double max_area = 0;
    int max_idx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    
    // 计算最大轮廓的质心
    cv::Moments m = cv::moments(contours[max_idx]);
    if (m.m00 <= 0) {
        return cv::Point(-1, -1);
    }
    
    int cx = static_cast<int>(m.m10 / m.m00);
    int cy = static_cast<int>(m.m01 / m.m00);*/
    
    // 直接计算整个ROI区域的质心
    cv::Moments m = cv::moments(red_mask, true);
    if (m.m00 <= 0) {
        return cv::Point(-1, -1);
    }
    
    int cx = static_cast<int>(m.m10 / m.m00);
    int cy = static_cast<int>(m.m01 / m.m00);
    
    mask = red_mask.clone();
    cv::resize(red_mask, red_mask, cv::Size(640,480));
    //cv::imshow("red", red_mask);
    
    return cv::Point(cx, cy);
}

//计算点云的欧拉角姿态（yaw→pitch→roll）
Eigen::Vector3f Algo::computeOrientationFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // 检查点云有效性
    if (cloud->empty() || cloud->size() < 3) {
        std::cerr << "Error: Invalid point cloud (size=" << cloud->size() << ")" << std::endl;
        return Eigen::Vector3f::Zero();
    }

    // 计算点云质心
    Eigen::Vector4f centroid;

    pcl::compute3DCentroid(*cloud, centroid);

    // 构建协方差矩阵
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

    // 进行特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success) {
        std::cerr << "Error: Eigen decomposition failed!" << std::endl;
        return Eigen::Vector3f::Zero();
    }

    // 特征向量按列排列，按特征值从小到大排列
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();

    // 主方向为最大特征值对应的特征向量，即最后一列
    // 为稳定性构造完整旋转矩阵（3个正交单位向量）
    Eigen::Vector3f principal_direction = eigenvectors.col(2);
    Eigen::Vector3f secondary_direction = eigenvectors.col(1);
    Eigen::Vector3f third_direction = eigenvectors.col(0);

    Eigen::Matrix3f rotation;
    rotation.col(0) = principal_direction.normalized(); // X轴
    rotation.col(1) = secondary_direction.normalized(); // Y轴
    rotation.col(2) = third_direction.normalized();     // Z轴

    // 确保右手坐标系
    if (rotation.determinant() < 0) {
        rotation.col(2) *= -1.0f;
    }

    // 计算欧拉角（ZYX顺序：yaw, pitch, roll）
    Eigen::Vector3f euler_angles;
    euler_angles[0] = std::atan2(rotation(1, 0), rotation(0, 0));       // Yaw
    euler_angles[1] = -std::asin(rotation(2, 0));                       // Pitch
    euler_angles[2] = std::atan2(rotation(2, 1), rotation(2, 2));       // Roll

    // 转换为角度制
    //euler_angles = euler_angles * 180.0f / M_PI;

    // 输出
    /*std::cout << "Orientation (Yaw, Pitch, Roll): "
              << euler_angles.transpose() << " degrees" << std::endl;*/

    return euler_angles;
}


/*函数：通过手眼标定矩阵转换3D位置和欧拉角姿态
 输入：
   - H_hand_eye: 4x4手眼标定矩阵（从相机到机械臂的变换）
   - point3d_cam: 相机坐标系下的3D点
   - euler_angles_cam: 相机坐标系下的欧拉角(Yaw, Pitch, Roll，单位弧度)
 输出：
   - point3d_arm: 机械臂坐标系下的3D点
   - euler_angles_arm: 机械臂坐标系下的欧拉角*/
void Algo::transformHandEye(const cv::Mat& H_hand_eye, 
                     const cv::Point3f& point3d_cam,
                     const Eigen::Vector3f& euler_angles_cam,
                     cv::Point3f& point3d_arm,
                     Eigen::Vector3f& euler_angles_arm) {
                     
    // 1. 确认矩阵方向（若H_hand_eye是机械臂基→相机，需求逆）
    cv::Mat H_cam_to_arm = H_hand_eye.clone(); // 默认方向正确
    //cv::Mat H_cam_to_arm = H_hand_eye.inv(); // 若方向反了
    
    // 确保输入矩阵是4x4
    CV_Assert(H_hand_eye.rows == 4 && H_hand_eye.cols == 4);

    // 1. 转换3D点坐标
    cv::Mat point_cam = (cv::Mat_<double>(4,1) << 
                         point3d_cam.x, 
                         point3d_cam.y, 
                         point3d_cam.z, 
                         1.0);
    cv::Mat point_arm = H_cam_to_arm * point_cam;
    point3d_arm = cv::Point3f(point_arm.at<double>(0),
                             point_arm.at<double>(1),
                             point_arm.at<double>(2));

    // 2. 转换欧拉角姿态
    // 2.1 将欧拉角转换为旋转矩阵
    Eigen::AngleAxisf rollAngle(euler_angles_cam(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(euler_angles_cam(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(euler_angles_cam(2), Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R_cam = (rollAngle * pitchAngle * yawAngle).toRotationMatrix();

    // 2.2 手动将OpenCV矩阵转换为Eigen矩阵
    Eigen::Matrix3f R_he;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_he(i, j) = static_cast<float>(H_hand_eye.at<double>(i, j));
        }
    }
    /*Eigen::Vector3f euler_angles_R_he(0,0,90.0/180*3.1415926); 
    Eigen::AngleAxisf rollAngle1(euler_angles_R_he(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle1(euler_angles_R_he(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle1(euler_angles_R_he(2), Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R_he1 = (rollAngle1 * pitchAngle1 * yawAngle1).toRotationMatrix();
    euler_angles_R_he = R_he1.eulerAngles(0, 1, 2); 
    
    Eigen::Vector3f euler_angles_R_cam = R_cam.eulerAngles(0, 1, 2); 
    std::cout << "euler_angles_R_he: "
              << euler_angles_R_he.transpose() /3.14*180 << " deg" << std::endl;
    std::cout << "euler_angles_R_cam: "
              << euler_angles_R_cam.transpose() /3.14*180  << " deg" << std::endl;    */      
    // 2.3 应用旋转变换
    Eigen::Matrix3f R_arm = R_he * R_cam;

    // 2.4 转换回欧拉角 (Yaw, Pitch, Roll)
    euler_angles_arm = R_arm.eulerAngles(0, 1, 2); // X,Y,Z轴对应Roll,Pitch,Yaw

}

