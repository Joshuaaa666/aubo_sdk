#!/usr/bin/env python
# coding=utf-8
"""
控制机械臂移动到指定的笛卡尔坐标
参数: x, y, z 坐标值（单位：米）
"""

import sys
import time
import math

try:
    import libpyauboi5
except ImportError as e:
    print "导入libpyauboi5失败: " + str(e)
    print "请确保Aubo SDK Python库已正确安装"
    sys.exit(1)

RS_SUCC = 0

def main(target_x, target_y, target_z, shutdown_after_move=False):
    print "尝试移动到位置: X=%.6f, Y=%.6f, Z=%.6f 米" % (target_x, target_y, target_z)
    
    # 初始化libpyauboi5库
    result = libpyauboi5.initialize()
    if result != 0:
        print "初始化失败"
        return False

    # 创建robot service控制上下文
    RSHD = libpyauboi5.create_context()
    if RSHD == -1:
        print "创建上下文失败"
        libpyauboi5.uninitialize()
        return False

    # 登陆服务器
    ip_addr = "192.168.10.100"  # 与test_1.py中的IP一致
    port = 8899

    result = libpyauboi5.login(RSHD, ip_addr, port)
    if result != 0:
        print "登录失败: %s:%d" % (ip_addr, port)
        libpyauboi5.destory_context(RSHD)
        libpyauboi5.uninitialize()
        return False

    print "登录成功"

    try:
        # 获取当前位姿，用于逆运动学计算
        current_waypoint = libpyauboi5.get_current_waypoint(RSHD)
        if not current_waypoint:
            print "无法获取当前位姿"
            return False

        print "当前位姿: pos=%s, ori=%s" % (str(current_waypoint['pos']), str(current_waypoint['ori']))

        # 当前关节角作为逆解的初始猜测
        current_joint = current_waypoint['joint']

        # 目标位置（单位：米
        target_pos = [target_x , target_y, target_z ]  
        
        # 目标姿态（暂时保持当前姿态）
        target_ori = current_waypoint['ori']

        print "目标位置(米): %s" % str(target_pos)
        print "目标姿态: %s" % str(target_ori)

        # 计算逆运动学
        ik_result = libpyauboi5.inverse_kin(RSHD, current_joint, target_pos, target_ori)
        if not ik_result or 'joint' not in ik_result:
            print "逆运动学计算失败"
            print "目标位置可能超出工作空间: %s" % str(target_pos)
            return False

        target_joint = ik_result['joint']
        print "计算得到的目标关节角: %s" % str(target_joint)

        # 启动机器人（这是关键步骤，需要设置碰撞等级等）
        tool_dynamics = {"position": (0, 0, 0), "payload": 1.0, "inertia": (0, 0, 0, 0, 0, 0)}
        collision = 6
        result = libpyauboi5.robot_startup(RSHD, collision, tool_dynamics)
        if result != 0:
            print "机器人启动失败，错误代码: %d" % result
            return False
        print "机器人启动成功"

        # 初始化全局运动配置
        libpyauboi5.init_global_move_profile(RSHD)

        # 移动到目标位置
        result = libpyauboi5.move_joint(RSHD, target_joint)
        if result == 0:
            print "机械臂移动成功"
        else:
            print "机械臂移动失败，错误代码: %d" % result
            return False

        # 等待移动完成
        time.sleep(2)

        # 默认不在每次运动后关闭机器人，避免看起来像“断电”。
        if shutdown_after_move:
            libpyauboi5.robot_shutdown(RSHD)
            print "已按参数请求关闭机器人"

    except Exception as e:
        print "移动过程中发生错误: %s" % str(e)
        return False

    finally:
        # 登出
        libpyauboi5.logout(RSHD)
        # 删除上下文
        libpyauboi5.destory_context(RSHD)
        # 反初始化库
        libpyauboi5.uninitialize()

    return True

if __name__ == "__main__":
    if len(sys.argv) not in (4, 5):
        print "Usage: python move_to_position.py <x> <y> <z> [--shutdown]"
        print "参数: x, y, z 坐标值（单位：米）"
        sys.exit(1)

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        
        shutdown_after_move = (len(sys.argv) == 5 and sys.argv[4] == "--shutdown")
        success = main(x, y, z, shutdown_after_move)
        if success:
            print "任务完成"
        else:
            print "任务失败"
            sys.exit(1)
    except ValueError:
        print "参数必须是数字"
        sys.exit(1)