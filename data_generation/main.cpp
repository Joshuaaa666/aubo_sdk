#include "kinect_camera.h"
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <memory>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <cstdlib>
#include <tuple>
#include <cstring>
#include <cerrno>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>

#ifdef USE_AUBO_SDK
#include <aubo_sdk/rpc.h>
#endif

#include <string>
#include <vector>
#include <sstream>

// 解析从示教器发来的关节角字符串
// 输入格式: "q_current:j1,j2,j3,j4,j5,j6;q_goal:j1,j2,j3,j4,j5,j6"
bool parseJointReply(const std::string& msg,
                     std::vector<double>& q_current,
                     std::vector<double>& q_goal)
{
    q_current.clear();
    q_goal.clear();

    // 1. 按分号分割两部分
    size_t sep = msg.find(';');
    if (sep == std::string::npos) return false;

    std::string part_qc = msg.substr(0, sep);
    std::string part_qg = msg.substr(sep + 1);

    // 2. 提取q_current的数值部分
    size_t pos_qc = part_qc.find(':');
    if (pos_qc == std::string::npos) return false;
    std::string str_qc = part_qc.substr(pos_qc + 1);

    // 3. 提取q_goal的数值部分
    size_t pos_qg = part_qg.find(':');
    if (pos_qg == std::string::npos) return false;
    std::string str_qg = part_qg.substr(pos_qg + 1);

    // 4. 字符串转double数组
    auto str2vec = [](const std::string& s) -> std::vector<double> {
        std::vector<double> v;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, ',')) {
            v.push_back(std::stod(item));
        }
        return v;
    };

    q_current = str2vec(str_qc);
    q_goal    = str2vec(str_qg);

    return (q_current.size() == 6 && q_goal.size() == 6);
}



namespace {

constexpr int kDof = 6;

std::string buildSampleStem(const std::string& prefix) {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm_now{};
    localtime_r(&t, &tm_now);

    std::ostringstream oss;
    oss << prefix << "_"
        << std::put_time(&tm_now, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

std::string vecToListString(const std::vector<double>& v) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i > 0) {
            oss << ",";
        }
        oss << std::setprecision(10) << v[i];
    }
    oss << "]";
    return oss.str();
}

// 手眼标定矩阵：Tool(E) <- Camera(C)。
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

bool useTcpOnlyMode() {
    const char* mode = std::getenv("DATAGEN_USE_TCP_ONLY");
    if (!mode) {
        return true;
    }
    std::string value(mode);
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    return !(value == "0" || value == "false" || value == "off" || value == "no");
}

Eigen::Vector3d rotationMatrixToRotvec(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}

bool sendPoseViaTcpAndWaitFinish(const std::vector<double>& target_pose,
                                 std::string* final_reply) {
    if (target_pose.size() < 6) {
        return false;
    }

    const char* ip_env = std::getenv("AUBO_IP");
    const char* port_env = std::getenv("AUBO_TCP_PORT");
    const char* rpc_port_env = std::getenv("AUBO_RPC_PORT");
    const std::string ip = ip_env ? ip_env : "192.168.10.100";
    const int port = port_env ? std::stoi(port_env) : (rpc_port_env ? std::stoi(rpc_port_env) : 8088);

    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "TCP: socket 创建失败" << std::endl;
        return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "TCP: 无效IP地址: " << ip << std::endl;
        ::close(sock);
        return false;
    }

    std::cout << "TCP: connecting to " << ip << ":" << port << std::endl;
    if (::connect(sock, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        std::cerr << "TCP: 连接失败" << std::endl;
        ::close(sock);
        return false;
    }

    const char* wait_env = std::getenv("AUBO_TCP_WAIT_SECONDS");
    const int wait_seconds = wait_env ? std::max(1, std::stoi(wait_env)) : 45;

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    std::ostringstream pose_ss;
    pose_ss << std::fixed << std::setprecision(6)
            << target_pose[0] << "," << target_pose[1] << "," << target_pose[2] << ","
            << target_pose[3] << "," << target_pose[4] << "," << target_pose[5];
    const char* append_newline_env = std::getenv("AUBO_TCP_APPEND_NEWLINE");
    const bool append_newline = append_newline_env && std::string(append_newline_env) == "1";
    const std::string payload = append_newline ? (pose_ss.str() + "\n") : pose_ss.str();

    if (::send(sock, payload.c_str(), payload.size(), 0) < 0) {
        std::cerr << "TCP: 发送失败" << std::endl;
        ::close(sock);
        return false;
    }

    std::cout << "TCP: sent pose=" << payload << std::endl;

        char buffer[512] = {0};
    std::string full_msg;  // 用来拼接所有收到的包
    bool got_finish = false;

    for (int i = 0; i < wait_seconds; ++i) {
        const int n = ::recv(sock, buffer, sizeof(buffer) - 1, 0);
        if (n == 0) {
            std::cout << "TCP: 对端已关闭连接" << std::endl;
            break;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                cv::waitKey(1);
                std::cout << "TCP: waiting Finish..." << std::endl;
                continue;
            }
            std::cout << "TCP: recv error errno=" << errno << std::endl;
            break;
        }

        cv::waitKey(1);
        buffer[n] = '\0';
        std::string msg(buffer);
        full_msg += msg;  // ✅ 关键：拼接所有收到的数据

        if (final_reply) {
            *final_reply = full_msg;
        }
        std::cout << "TCP: recv=" << msg << " | full=" << full_msg << std::endl;

        // ✅ 收到 Finish 才退出
        if (msg.find("Finish") != std::string::npos || msg.find("finish") != std::string::npos) {
            got_finish = true;
            break;
        }
    }

    if (got_finish) {
        std::cout << "TCP: 收到 Finish，机械臂已完成" << std::endl;
    } else {
        std::cout << "TCP: 超时未收到 Finish" << std::endl;
    }

    // ✅ 用拼接完的完整消息解析
    std::vector<double> q_current, q_goal;
    if (!full_msg.empty() && parseJointReply(full_msg, q_current, q_goal)) {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "✅ 关节角解析成功！" << std::endl;
        std::cout << "q_current(起点): " << vecToListString(q_current) << std::endl;
        std::cout << "q_goal(终点):    " << vecToListString(q_goal) << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }

    ::close(sock);
    return got_finish;

    ::close(sock);
    return false;
}

#ifdef USE_AUBO_SDK
struct AuboSession {
    arcs::aubo_sdk::RpcClientPtr rpc;
    std::string robot_name;
    std::chrono::steady_clock::time_point next_retry_time = std::chrono::steady_clock::time_point::min();
    bool retry_notice_printed = false;
};

AuboSession& getAuboSession() {
    static AuboSession session;
    return session;
}

bool ensureAuboSessionReady() {
    try {
        auto& session = getAuboSession();
        const auto now = std::chrono::steady_clock::now();
        if (now < session.next_retry_time) {
            if (!session.retry_notice_printed) {
                std::cout << "AUBO: 上次连接失败，5秒后自动重试，当前跳过本次连接。" << std::endl;
                session.retry_notice_printed = true;
            }
            return false;
        }

        if (session.rpc) {
            return true;
        }

        session.rpc = std::make_shared<arcs::aubo_sdk::RpcClient>();
        session.rpc->setRequestTimeout(300);

        const char* ip_env = std::getenv("AUBO_IP");
        const char* user_env = std::getenv("AUBO_USER");
        const char* pass_env = std::getenv("AUBO_PASS");
        const char* port_env = std::getenv("AUBO_RPC_PORT");

        const std::string ip = ip_env ? ip_env : "192.168.10.100";
        const int port = port_env ? std::stoi(port_env) : 8088;

        if (session.rpc->connect(ip, port) != 0) {
            std::cerr << "AUBO RPC 连接失败，请检查 AUBO_IP 与端口。" << std::endl;
            session.rpc.reset();
            session.next_retry_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            session.retry_notice_printed = false;
            return false;
        }

        // 免登录控制器可不设置账号；如设置了账号密码，则尝试登录。
        const bool has_user = user_env && std::strlen(user_env) > 0;
        const bool has_pass = pass_env && std::strlen(pass_env) > 0;
        if (has_user || has_pass) {
            if (!(has_user && has_pass)) {
                std::cerr << "请同时设置 AUBO_USER 与 AUBO_PASS，或都不设置(免登录)。" << std::endl;
                session.rpc.reset();
            session.next_retry_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            session.retry_notice_printed = false;
            return false;
            }
            if (session.rpc->login(user_env, pass_env) != 0) {
                std::cerr << "AUBO RPC 登录失败，请检查 AUBO_USER/AUBO_PASS。" << std::endl;
                session.rpc.reset();
            session.next_retry_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            session.retry_notice_printed = false;
            return false;
            }
        } else {
            std::cout << "AUBO: 未设置账号密码，按免登录模式连接。" << std::endl;
        }

        const auto names = session.rpc->getRobotNames();
        if (names.empty()) {
            std::cerr << "AUBO RPC 未返回机器人名称。" << std::endl;
            session.rpc.reset();
            session.next_retry_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            session.retry_notice_printed = false;
            return false;
        }

        session.robot_name = names.front();
        session.retry_notice_printed = false;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "AUBO RPC 初始化异常: " << e.what() << std::endl;
        auto& session = getAuboSession();
        session.rpc.reset();
        session.next_retry_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        session.retry_notice_printed = false;
        return false;
    }
}
#endif

// 读取机器人实时状态：Base->Tool 位姿、当前关节角、当前 TCP 位姿。
bool getBaseToToolFromRobot(Eigen::Matrix4d& base_T_tool,
                            std::vector<double>* q_start,
                            std::vector<double>* tcp_pose) {
#ifdef USE_AUBO_SDK
    if (!ensureAuboSessionReady()) {
        return false;
    }

    try {
        auto& session = getAuboSession();
        auto robot = session.rpc->getRobotInterface(session.robot_name);

        const auto pose = robot->getRobotState()->getTcpPose();
        const auto q = robot->getRobotState()->getJointPositions();

        if (pose.size() < 6 || q.size() < kDof) {
            std::cerr << "AUBO 读取状态长度异常。" << std::endl;
            return false;
        }

        if (q_start) {
            *q_start = q;
        }
        if (tcp_pose) {
            *tcp_pose = pose;
        }

        base_T_tool = poseToMatrix(pose);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "AUBO 读取状态异常: " << e.what() << std::endl;
        return false;
    }
#else
    (void)base_T_tool;
    (void)q_start;
    (void)tcp_pose;
    return false;
#endif
}

// 根据目标点 center_base 生成目标 TCP 位姿并调用官方 IK。
bool computeIkGoalFromTarget(const std::vector<double>& q_start,
                             const std::vector<double>& tcp_pose_now,
                             const Eigen::Vector3d& center_base,
                             std::vector<double>& q_goal_out) {
#ifdef USE_AUBO_SDK
    if (!ensureAuboSessionReady()) {
        return false;
    }
    if (q_start.size() < kDof || tcp_pose_now.size() < 6) {
        return false;
    }

    try {
        auto& session = getAuboSession();
        auto robot = session.rpc->getRobotInterface(session.robot_name);

        // 目标位姿：平移改为 center_base，姿态沿用当前 TCP。
        std::vector<double> target_pose = tcp_pose_now;
        target_pose[0] = center_base.x();
        target_pose[1] = center_base.y();
        target_pose[2] = center_base.z();

        auto ik_ret = robot->getRobotAlgorithm()->inverseKinematics(q_start, target_pose);
        const auto& q_goal = std::get<0>(ik_ret);
        const int ik_errno = std::get<1>(ik_ret);

        if (ik_errno != 0 || q_goal.size() < kDof) {
            std::cerr << "IK 失败，错误码=" << ik_errno << std::endl;
            return false;
        }

        q_goal_out = q_goal;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "IK 调用异常: " << e.what() << std::endl;
        return false;
    }
#else
    (void)q_start;
    (void)tcp_pose_now;
    (void)center_base;
    (void)q_goal_out;
    return false;
#endif
}

bool computeCloudCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3d& center_cam) {
    if (!cloud || cloud->empty()) {
        return false;
    }

    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*cloud, centroid4);
    center_cam = centroid4.head<3>().cast<double>();
    return true;
}

Eigen::Vector3d transformCamPointToBase(const Eigen::Vector3d& point_cam,
                                        const Eigen::Matrix4d& base_T_tool,
                                        const Eigen::Matrix4d& tool_T_cam) {
    Eigen::Vector4d p_cam(point_cam.x(), point_cam.y(), point_cam.z(), 1.0);
    Eigen::Vector4d p_base = base_T_tool * tool_T_cam * p_cam;
    return p_base.head<3>();
}

void appendTrajectoryRecord(const std::string& csv_path,
                            const std::string& sample_id,
                            const cv::Rect& roi,
                            std::size_t point_count,
                            const Eigen::Vector3d& center_cam,
                            const Eigen::Vector3d& center_base,
                            bool has_base_pose,
                            const std::vector<double>& q_start,
                            const std::vector<double>& q_goal,
                            bool has_ik_goal,
                            bool tcp_sent,
                            bool tcp_finish) {
    const bool file_exists = std::filesystem::exists(csv_path);
    std::ofstream ofs(csv_path, std::ios::app);
    if (!ofs.is_open()) {
        std::cerr << "无法写入轨迹记录文件: " << csv_path << std::endl;
        return;
    }

    if (!file_exists) {
        ofs << "sample_id,roi_x,roi_y,roi_w,roi_h,point_count,"
               "center_cam_x,center_cam_y,center_cam_z,"
               "center_base_x,center_base_y,center_base_z,has_base_pose,"
               "planner,status,start_pose,goal_pose,joint_path,has_ik_goal,tcp_sent,tcp_finish\n";
    }

    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const double bx = has_base_pose ? center_base.x() : nan;
    const double by = has_base_pose ? center_base.y() : nan;
    const double bz = has_base_pose ? center_base.z() : nan;

    ofs << sample_id << ","
        << roi.x << "," << roi.y << "," << roi.width << "," << roi.height << ","
        << point_count << ","
        << center_cam.x() << "," << center_cam.y() << "," << center_cam.z() << ","
        << bx << "," << by << "," << bz << "," << (has_base_pose ? 1 : 0) << ","
        << "RRT_PLACEHOLDER,"
        << (has_ik_goal ? "IK_READY" : "IK_PENDING") << ","
        << vecToListString(q_start) << ","
        << vecToListString(q_goal) << ","
        << "[]"
        << "," << (has_ik_goal ? 1 : 0)
        << "," << (tcp_sent ? 1 : 0)
        << "," << (tcp_finish ? 1 : 0)
        << "\n";
}

}  // namespace

int main() {
    std::unique_ptr<KinectCamera> kinect = std::make_unique<KinectCamera>();
    if (!kinect->initialize()) {
        return -1;
    }

    const Eigen::Matrix4d tool_T_cam = buildToolToCamera();

    cv::Mat frame;
    cv::Mat raw_frame;
    cv::Mat depth_color;
    cv::Mat aligned_depth;
    cv::Mat aligned_depth_color;

    cv::Rect resultBox;
    bool has_roi = false;

    std::filesystem::path output_dir_path;
    const auto cwd = std::filesystem::current_path();
    if (cwd.filename() == "build") {
        output_dir_path = cwd.parent_path() / "dataset";
    } else {
        output_dir_path = cwd / "dataset";
    }
    std::filesystem::create_directories(output_dir_path);

    const std::string output_dir = output_dir_path.string();
    const std::string csv_path = (output_dir_path / "trajectory_records_v2.csv").string();

    std::cout << "--BEGIN--\n" << std::endl;
    std::cout << "按键说明: r=选择目标框, s=保存ROI点云+轨迹记录, ESC=退出\n";
    std::cout << "提示: 先点击 RGB 窗口再按 r/s，SSH远程桌面下可能有键盘焦点问题。\n";
    std::cout << "数据保存目录: " << output_dir << std::endl;
    std::cout << "CSV路径: " << csv_path << std::endl;
    const bool tcp_only_mode = useTcpOnlyMode();
    std::cout << "控制模式: " << (tcp_only_mode ? "TCP-only" : "SDK/RPC") << std::endl;
    int consecutive_frame_failures = 0;
    constexpr int kMaxConsecutiveFrameFailures = 30;

#ifndef USE_AUBO_SDK
    std::cout << "当前未启用 USE_AUBO_SDK，center_base 与 IK 相关字段将记录为占位值。\n";
#endif
    while (true) {
        // 
        
        // ✅ 只取帧，失败就延时，绝不重连、绝不重建对象
    if (!kinect->getNextFrame(frame, depth_color, &aligned_depth, &aligned_depth_color)) {
        std::cout << "get Frame failed, retry..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
    }

    if (frame.empty()) {
        continue;
    }

    consecutive_frame_failures = 0;
    raw_frame = frame.clone();

    if (has_roi) {
        cv::rectangle(frame, resultBox, cv::Scalar(0, 255, 0), 2);
        if (!aligned_depth_color.empty()) {
            cv::rectangle(aligned_depth_color, resultBox, cv::Scalar(0, 255, 0), 2);
        }
    }

    cv::imshow("RGB", frame);
    if (std::getenv("SHOW_ORIGINAL_DEPTH")) {
        cv::imshow("Original Depth", depth_color);
    }
    cv::imshow("Aligned Depth", aligned_depth_color);

    const int key_raw = cv::waitKeyEx(20);
    const int key = (key_raw >= 0) ? (key_raw & 0xFF) : -1;
    if (key_raw >= 0 && key_raw != key) {
        // 某些后端会在高位附带扫描码，统一截断到低8位做按键判断。
        std::cout << "key_raw=" << key_raw << ", key=" << key << std::endl;
    }

        if (key == 'r' || key == 'R') {
            cv::Rect selected = cv::selectROI("RGB", raw_frame, false, false);
            if (selected.width > 0 && selected.height > 0) {
                resultBox = selected;
                has_roi = true;
                std::cout << "ROI selected: ["
                          << resultBox.x << ", " << resultBox.y << ", "
                          << resultBox.width << ", " << resultBox.height << "]\n";
            } else {
                std::cout << "ROI selection canceled." << std::endl;
            }
        }

        if (key == 's' || key == 'S') {
            if (!has_roi) {
                std::cout << "请先按 r 选择目标框。" << std::endl;
                continue;
            }
            if (aligned_depth.empty()) {
                std::cout << "aligned_depth is empty, skip saving." << std::endl;
                continue;
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (!kinect->depthImageToPointCloudInROI(aligned_depth, resultBox, roi_cloud, raw_frame)) {
                std::cout << "ROI 点云生成失败。" << std::endl;
                continue;
            }

            auto target_cloud = kinect->extractLargestEuclideanCluster(roi_cloud, 0.01, 80, 300000);
            if (!target_cloud || target_cloud->empty()) {
                target_cloud = roi_cloud;
            }

            Eigen::Vector3d center_cam;
            if (!computeCloudCentroid(target_cloud, center_cam)) {
                std::cout << "无法计算点云质心。" << std::endl;
                continue;
            }

            Eigen::Matrix4d base_T_tool = Eigen::Matrix4d::Identity();
            std::vector<double> q_start(kDof, 0.0);
            std::vector<double> tcp_pose_now(kDof, 0.0);
            Eigen::Vector3d center_base = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            std::vector<double> q_goal(kDof, 0.0);
            bool has_ik_goal = false;
            bool has_base_pose = false;
            bool tcp_sent = false;
            bool tcp_finish = false;

            if (tcp_only_mode) {
                // TCP-only: 假定相机固定，直接使用固定外参将相机点转换到机械臂基座系。
                has_base_pose = true;
                center_base = transformCamPointToBase(center_cam, base_T_tool, tool_T_cam);

                const Eigen::Vector3d rotvec = rotationMatrixToRotvec(tool_T_cam.block<3, 3>(0, 0));
                std::vector<double> target_pose = {
                    center_base.x(), center_base.y(), center_base.z(),
                    rotvec.x(), rotvec.y(), rotvec.z()
                };

                std::string tcp_reply;
                tcp_finish = sendPoseViaTcpAndWaitFinish(target_pose, &tcp_reply);
                tcp_sent = true;
                if (!tcp_finish) {
                    std::cout << "TCP: 未收到 Finish，机械臂可能未执行完成。" << std::endl;
                }
            } else {
                has_base_pose = getBaseToToolFromRobot(base_T_tool, &q_start, &tcp_pose_now);
                if (has_base_pose) {
                    center_base = transformCamPointToBase(center_cam, base_T_tool, tool_T_cam);
                    has_ik_goal = computeIkGoalFromTarget(q_start, tcp_pose_now, center_base, q_goal);
                } else {
                    std::cout << "未读取到机器人 Base->Tool 位姿，center_base 先记为 NaN。" << std::endl;
                }
            }

            const std::string sample_id = buildSampleStem("target");
            const std::string pcd_path = output_dir + "/" + sample_id + ".pcd";
            const std::string ply_path = output_dir + "/" + sample_id + ".ply";
            const std::string obj_path = output_dir + "/" + sample_id + ".obj";

            PointCloudSaver::Save(target_cloud, pcd_path);
            PointCloudSaver::Save(target_cloud, ply_path);
            PointCloudSaver::Save(target_cloud, obj_path, false);

            appendTrajectoryRecord(csv_path,
                                   sample_id,
                                   resultBox,
                                   target_cloud->size(),
                                   center_cam,
                                   center_base,
                                   has_base_pose,
                                   q_start,
                                   q_goal,
                                   has_ik_goal,
                                   tcp_sent,
                                   tcp_finish);

            std::cout << "Saved sample: " << sample_id
                      << " | points=" << target_cloud->size()
                      << " | center_cam(m)=[" << center_cam.x() << ", " << center_cam.y() << ", " << center_cam.z() << "]";
            if (has_base_pose) {
                std::cout << " | center_base(m)=[" << center_base.x() << ", " << center_base.y() << ", " << center_base.z() << "]";
            }
            if (has_ik_goal) {
                std::cout << " | IK goal ready";
            }
            if (tcp_only_mode) {
                std::cout << " | tcp_sent=" << (tcp_sent ? 1 : 0)
                          << " tcp_finish=" << (tcp_finish ? 1 : 0);
            }
            std::cout << std::endl;
        }

        if (key == 27) {
            break;
        }
    }

    kinect->shutdown();
    cv::destroyAllWindows();
    return 0;
}
