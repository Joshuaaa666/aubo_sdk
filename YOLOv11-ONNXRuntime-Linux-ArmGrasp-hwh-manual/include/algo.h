#include "engine.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iostream>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>

class Algo
{
public:
	Algo();
	~Algo();
	void init();
	bool inferResult(cv::Mat& img, cv::Rect& highestBox);

	void setModePath(const std::string mode_str);
	
	void setStride(const int stride);
	
	cv::Point calculateCentroid(cv::Mat& image, const cv::Rect& box, cv::Mat& mask);
	
	Eigen::Vector3f computeOrientationFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	
	void transformHandEye(const cv::Mat& H_hand_eye, 
                     const cv::Point3f& point3d_cam,
                     const Eigen::Vector3f& euler_angles_cam,
                     cv::Point3f& point3d_arm,
                     Eigen::Vector3f& euler_angles_arm);
	
private:
	std::shared_ptr<YoloInferencer> inferencer_;
	std::string model_path_;
	const char* logid_ ;
	const char* provider_; // "CPU" or "CUDA"
	
};

