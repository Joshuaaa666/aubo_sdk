#! /usr/bin/env python
# coding=utf-8
import time
import math
import libpyauboi5

RS_SUCC = 0

print(__name__)

# 获取系统当前时间
def get_local_time():
    return time.strftime("%b %d %Y %H:%M:%S:", time.localtime(time.time()))

def get_local_mes_time():
    ct = time.time()
    data_msecs = (ct-int(ct))*1000
    time_msecs = str("%03d" % data_msecs)
    return time_msecs

# 四元数转欧拉角 (ZYX顺序，即yaw-pitch-roll顺序)
def quaternion_to_rpy(w, x, y, z):
    # 计算roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # 计算pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用1进行钳位
    else:
        pitch = math.asin(sinp)

    # 计算yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# 机械臂事件
def robot_event(event):
    print("catch event{0}".format(event))

def main():
    # 初始化libpyauboi5库
    result = libpyauboi5.initialize()
    print("{0} initialize result={1}".format(get_local_time(), result))

    # 创建robot service控制上下文
    RSHD = libpyauboi5.create_context()
    print("create_context RSHD={0}".format(RSHD))

    # 登陆服务器
    #ip_addr = "127.0.0.1"
    ip_addr = "192.168.10.100"
    port = 8899

    print("{0}{1}-------log start".format(get_local_time(),get_local_mes_time()))
    

    result = libpyauboi5.login(RSHD, ip_addr, port)

    if result == RS_SUCC:
        # 登陆成功
        print("login {0}:{1} succ.".format(ip_addr, port))

        print("{0}{1}-------log end".format(get_local_time(),get_local_mes_time()))

        # 获取关节当前位置
        waypoint = libpyauboi5.get_current_waypoint(RSHD)
        print("currrent waypoint={0}".format(waypoint))

        # 提取并格式化显示坐标
        if waypoint:
            joint = waypoint['joint']
            pos = waypoint['pos']
            ori = waypoint['ori']  # 四元数 (w, x, y, z)
            
            print("\n=== 当前机械臂位置 ===")
            print("关节角度(弧度): [{0:.4f}, {1:.4f}, {2:.4f}, {3:.4f}, {4:.4f}, {5:.4f}]".format(*joint))
            print("笛卡尔坐标(米): X={0:.4f}, Y={1:.4f}, Z={2:.4f}".format(*pos))
            print("四元数姿态: W={0:.4f}, X={1:.4f}, Y={2:.4f}, Z={3:.4f}".format(ori[0], ori[1], ori[2], ori[3]))
            
            # 将四元数转换为欧拉角
            roll, pitch, yaw = quaternion_to_rpy(ori[0], ori[1], ori[2], ori[3])
            print("欧拉角姿态(弧度): RX={0:.4f}, RY={1:.4f}, RZ={2:.4f}".format(roll, pitch, yaw))
            print("欧拉角姿态(角度): RX={0:.2f}, RY={1:.2f}, RZ={2:.2f}".format(
                math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
            print("=====================\n")

    else:  # 登陆失败
        print("login {0}:{1} failed.".format(ip_addr, port))

    # 删除上下文
    libpyauboi5.destory_context(RSHD)

    # 反初始化libpyauboi5库
    libpyauboi5.uninitialize()


if __name__ == "__main__":

    count = 1
    while count < 2:
        print("************************mission start({0})*****************************".format(count))
        main()
        count += 1
        print("************************mission completed!************************")