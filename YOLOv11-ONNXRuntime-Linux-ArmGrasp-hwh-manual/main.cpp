#include "algo.h"
#include "kinect_camera.h"
#include "tcp_comm.h"
#include "serial.h"
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <memory>

int main() {
	std::cout << "=== 速度控制程序启动 ===" << std::endl;
    
    // 1. 创建串口对象并打开
    SerialPort serial;
    
    std::cout << "正在打开串口 /dev/lingao ..." << std::endl;
    if (!serial.open("/dev/lingao", 230400)) {
        std::cerr << "错误：无法打开串口！" << std::endl;
        return 1;
    }
	std::cout << "成功打开/dev/lingao ..." << std::endl;
	uint8_t buffer1[32];
    size_t data_length1;
    
    std::cout << "\n--- First Speed Command ---" << std::endl;
    SerialPort::createVelocityCommand(0.2f, 0.0f, 0.0f, buffer1, data_length1);
    std::cout << "Command length: " << data_length1 << " bytes" << std::endl;
    SerialPort::printHexData(buffer1, data_length1);
    SerialPort::validateVelocityCommand(buffer1, data_length1);
    
    uint8_t buffer2[32];
    size_t data_length2;
    
    std::cout << "\n--- First Speed Command ---" << std::endl;
    SerialPort::createVelocityCommand(-0.2f, 0.0f, 0.0f, buffer2, data_length2);
    std::cout << "Command length: " << data_length2 << " bytes" << std::endl;
    SerialPort::printHexData(buffer2, data_length2);
    SerialPort::validateVelocityCommand(buffer2, data_length2);
    /*//发送速度设置指令
    serial.send(buffer, data_length);
    
    std::cout << "\nWaiting 1 second..." << std::endl;
    sleep(1);

    //发送速度设置指令
    serial.send(buffer, data_length);
    
    std::cout << "\nWaiting 1 second..." << std::endl;
    sleep(1);

    //发送速度设置指令
    serial.send(buffer, data_length);*/
    
    /*for(int i=0;i<20;i++){
    	std::cout << i << std::endl;
    	std::cout << "moveing ...." << std::endl;
    	serial.send(buffer, data_length);
    	
		std::cout << "\nWaiting 1 second..." << std::endl;
		usleep(500*1000);
    }*/
    
    
	//return 0;

    std::string ip = "192.168.10.100";
	int port = 8088;
		    
	std::cout << "Starting TCP Communication Test" << std::endl;
	std::cout << "Server: " << ip << ":" << port << std::endl;
		    
	TCPClient client(ip, port);

	if(!client.connection_flag){
	    return -1;
	}

    // 创建并初始化Kinect相机
    std::unique_ptr<KinectCamera> kinect = std::make_unique<KinectCamera>();
    if (!kinect->initialize()) {
        return -1;
    }

    Algo algo;
    algo.setModePath("/home/lingao/yolo_cpp_rewrite/YOLOv11-ONNXRuntime-Linux-ArmGrasp-hwh/a3.onnx");
    algo.setStride(20);
    algo.init();
    
    cv::Mat frame;
    cv::Mat raw_frame;
    cv::Mat depth_color;
    cv::Mat mask;
    cv::Mat aligned_depth;        // 对齐后的深度数据（16位）
    cv::Mat aligned_depth_color;  // 对齐后的深度可视化
    cv::Mat blended;
    cv::Rect resultBox;  //检测目标框
    cv::Point3f point2d;
    cv::Point3f point3d;
    Eigen::Vector4f centroid;
    
    cv::Point3f point3d_arm;
    Eigen::Vector3f euler_angles_arm;
    cv::Mat H_hand_eye = (cv::Mat_<double>(4, 4) << 
        0.13551828522285903,  0.99077427969707599,  0.0010588016795823466, -135.91254527050771,
        -0.99077376072521717,  0.13551964081471257, -0.0013349201705961042,  -51.733981386034912,
        -0.0014660929937864665, -0.0008681268295133869, 0.99999854846251723,  -90.646138164038007,
        0.0, 0.0, 0.0, 1.0);  
    cv::Mat axis_remap = (cv::Mat_<double>(4,4) << 
        0,  0,  1,  0,
        1,  0,  0,  0,
        0, 1,  0,  0,
        0,  0,  0,  1);
    //H_hand_eye = H_hand_eye * axis_remap;

    // 检查矩阵是否正确初始化
    std::cout << "Hand-eye calibration matrix:\n" << H_hand_eye << std::endl;
    
    //go---
	for(int i=0;i<20;i++){
		std::cout << i << std::endl;
		std::cout << "go-moveing ...." << std::endl;
		serial.send(buffer1, data_length1);
		
		std::cout << "\nWaiting 500ms..." << std::endl;
		usleep(500*1000);
	}
	sleep(4);
    int flag = 0;
    while (true) {
    	
		
        // 从Kinect获取帧（包含对齐的深度图）
        if (kinect->getNextFrame(frame, depth_color, &aligned_depth, &aligned_depth_color)) {
            // 显示原始深度图
            //cv::imshow("Original Depth", depth_color);
            // 显示对齐后的深度图
            //cv::imshow("Aligned Depth", aligned_depth_color);
            
            if (frame.empty()) continue;
            raw_frame = frame.clone();
            
            // 使用对齐的深度数据进行处理，在这里，frame 和 aligned_depth 已经对齐
            if (!aligned_depth.empty()) {
                // 可以基于检测结果获取深度值
                bool res = algo.inferResult(frame, resultBox);
                if(!res){
                       std::cout<<"None Detection!!!"<<std::endl;
                	continue;
                }
                
                cv::Point keypoint = algo.calculateCentroid(raw_frame, resultBox, mask);

                aligned_depth.setTo(0, ~mask);

                cv::resize(aligned_depth_color,aligned_depth_color,frame.size());
                //cv::imshow("Aligned Depth", aligned_depth_color);
                
                cv::addWeighted(frame,0.7,aligned_depth_color,0.3,0,blended);   
                
                //可视化
                cv::resize(frame, frame, cv::Size(640, 480));
                //cv::imshow("Detection", frame);
                //cv::resize(aligned_depth_color,aligned_depth_color,frame.size());
                //cv::imshow("Aligned Depth", aligned_depth_color);
                cv::resize(blended,blended,frame.size());
                cv::imshow("Blended View", blended);
                //cv::resize(raw_frame,raw_frame,frame.size());
                //cv::imshow("raw frame",raw_frame);
                cv::waitKey(1000);
		 
			    //检测框中心
			    cv::Point center(resultBox.x + resultBox.width/2, resultBox.y + resultBox.height/2);
			    cv::circle(frame, center, 10, cv::Scalar(255, 0, 0), -1);
			 
			    //目标质心
			    std::cout<<"pointx="<<keypoint.x<<" pointy="<<keypoint.y<<std::endl;
			    cv::circle(frame, keypoint, 10, cv::Scalar(0, 0, 255), -1);
		 
				// 获取中心点深度值
				if (keypoint.x >= 0 && keypoint.x < aligned_depth.cols &&
				    keypoint.y >= 0 && keypoint.y < aligned_depth.rows) 
				{
				    uint16_t depth_value = aligned_depth.at<uint16_t>(keypoint);
				    
				    if (depth_value > 0) {
				        // 转换为3D坐标（彩色相机坐标系）
				        int valid = 0;
				        point2d = kinect->pixelToPoint3D(keypoint, depth_value, &valid);

				        //point3d.z /= 1000.0f ;
				        
				        std::cout << "2D质心("
				                      << point2d.x << ", " 
				                      << point2d.y << ", "
				                      << point2d.z << ") mm" << std::endl;
				    }
				}
        
        		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

				if (kinect->depthImageToPointCloud(aligned_depth, cloud, raw_frame)) {
				           
				    //过滤点云，提取最大簇(这个部分存在 段错误 (核心已转储) 的问题)
				    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr largest_cluster = kinect->extractLargestEuclideanCluster(cloud, 0.01, 50, 100000);*/
				    pcl::PointCloud<pcl::PointXYZRGB>::Ptr largest_cluster = cloud;
					
					//默认于相机姿态相同
				    Eigen::Vector3f euler_angles(0, 0, 0);  
				    std::cout<<"start"<<std::endl;		    
					
					//使用PCA取得目标点云的旋转姿态
					/*euler_angles = algo.computeOrientationFromPointCloud(largest_cluster);
					std::cout << "Orientation (Yaw, Pitch, Roll): "
				                      << euler_angles.transpose() << "rad" << std::endl;*/
					
					//取得质心
					pcl::compute3DCentroid(*largest_cluster, centroid);
					/*if (cloud->empty()) {
						return cv::Point3f(0.0, 0.0, 0.0);
					}*/
					
					// 使用max_element查找最高点
					/*auto max_it = std::max_element(largest_cluster->begin(), largest_cluster->end(),
						[](const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b) {
							return a.z < b.z;
						});*/
						
					point3d.x = centroid.x();
					point3d.y = centroid.y() - 0.11;
					point3d.z = centroid.z() - 0.09;
					/*point3d.x = max_it->x;
					point3d.y = max_it->y;
					point3d.z = max_it->z;*/
					std::cout << "6D位姿Cam("<< point3d.x << ", " 
				                     << point3d.y << ", "
				                     << point3d.z << ") m, " 
				                     << euler_angles[0] << ", "
				                     << euler_angles[1] << ", "
				                     << euler_angles[2] << "rad"
				                     << std::endl;
				                     
				    algo.transformHandEye(H_hand_eye,point3d*1000,euler_angles,point3d_arm,euler_angles_arm);
				    point3d_arm /= 1000;
				    std::cout << "6D位姿Arm("<< point3d_arm.x << ", " 
				                     << point3d_arm.y << ", "
				                     << point3d_arm.z << ") m, " 
				                     << euler_angles_arm[0] << ", "
				                     << euler_angles_arm[1] << ", "
				                     << euler_angles_arm[2] << "rad"
				                     << std::endl;
				                     
				           
					client.sendPose0(point3d_arm,euler_angles_arm);
					
					while(true) {
						if (client.waitForMsg("Finish")) {
							std::cout << "Successfully received Finish!" << std::endl;
							std::cout << flag++ << std::endl;
							break;
						} else {
							std::cerr << "Failed to receive Finish, Waiting..." << std::endl;
							std::this_thread::sleep_for(std::chrono::seconds(1));
							
						}
					}
					//sleep(3);
				                      
					// 可视化点云
					/*kinect->visualizePointCloud(
						largest_cluster, 
						"PointCloud with Pose",
						cv::Scalar(0, 0, 0), 
						point3d,                    
						euler_angles,                     
						0.10f,
						"camera_pose"); */           
				}              
            }
			if(flag == 3) break;
            // 按 ESC 退出
            if (cv::waitKey(30) == 27) break;
        } else {
            std::cout << "Frame capture failed" << std::endl;
            continue;
        }
    }
    
    
    //back---
	for(int i=0;i<20;i++){
		std::cout << i << std::endl;
		std::cout << "back-moveing ...." << std::endl;
		serial.send(buffer2, data_length2);
		
		std::cout << "\nWaiting 500ms..." << std::endl;
		usleep(500*1000);
	}
	sleep(3);
    
    kinect->shutdown();
    cv::destroyAllWindows();
    return 0;
}
