#! /usr/bin/env python
# coding=utf-8
import time
import libpyauboi5

RS_SUCC = 0

def get_local_time():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))

def move_to_position(RSHD, target_joint_angles):
    """
    让机械臂运动到指定的关节角度位置
    
    Args:
        RSHD: 机器人句柄
        target_joint_angles: 目标关节角度元组 (6个关节的弧度值)
    
    Returns:
        int: 执行结果，0表示成功
    """
    print("{0} 开始运动到目标位置...".format(get_local_time()))
    print("目标关节角度: [{0:.4f}, {1:.4f}, {2:.4f}, {3:.4f}, {4:.4f}, {5:.4f}]".format(
        *target_joint_angles))
    
    result = libpyauboi5.move_joint(RSHD, target_joint_angles)
    
    if result == RS_SUCC:
        print("{0} 运动完成".format(get_local_time()))
        
        # 获取运动后的位置
        waypoint = libpyauboi5.get_current_waypoint(RSHD)
        if waypoint:
            pos = waypoint['pos']
            print("当前位置: X={0:.4f}, Y={1:.4f}, Z={2:.4f}".format(
                pos[0], pos[1], pos[2]))
    else:
        print("{0} 运动失败，错误码: {1}".format(get_local_time(), result))
    
    return result

def main():
    # ========== 初始化机械臂位置 ==========
  
    
    target_joints = (1.5895, -0.4545, -2.1783, -1.5664, -1.5630, 0.7595)
    
    print("="*50)
    print("Aubo机械臂位置控制")
    print("="*50)
    
    # 初始化libpyauboi5库
    result = libpyauboi5.initialize()
    print("{0} 初始化结果: {1}".format(get_local_time(), result))

    # 创建robot service控制上下文
    RSHD = libpyauboi5.create_context()
    
    # 登录服务器
    ip_addr = "192.168.10.100"
    port = 8899
    
    result = libpyauboi5.login(RSHD, ip_addr, port)

    if result == RS_SUCC:
        print("{0} 连接成功".format(get_local_time()))
        
        # 启动机器人（这是关键步骤）
        tool_dynamics = {"position": (0, 0, 0), "payload": 1.0, "inertia": (0, 0, 0, 0, 0, 0)}
        collision = 6
        result = libpyauboi5.robot_startup(RSHD, collision, tool_dynamics)
        if result != RS_SUCC:
            print("{0} 机器人启动失败，错误码: {1}".format(get_local_time(), result))
            libpyauboi5.logout(RSHD)
            libpyauboi5.destory_context(RSHD)
            libpyauboi5.uninitialize()
            return
        print("{0} 机器人启动成功".format(get_local_time()))
        
        # 初始化全局运动配置
        libpyauboi5.init_global_move_profile(RSHD)
        
        # 获取当前位置
        print("\n--- 运动前位置 ---")
        waypoint = libpyauboi5.get_current_waypoint(RSHD)
        if waypoint:
            joint = waypoint['joint']
            pos = waypoint['pos']
            print("关节角度: [{0:.4f}, {1:.4f}, {2:.4f}, {3:.4f}, {4:.4f}, {5:.4f}]".format(*joint))
            print("笛卡尔坐标: X={0:.4f}, Y={1:.4f}, Z={2:.4f}".format(pos[0], pos[1], pos[2]))
        
        # 执行运动
        print("\n--- 开始运动 ---")
        move_to_position(RSHD, target_joints)
        
        # 再次获取位置确认
        print("\n--- 运动后位置 ---")
        waypoint = libpyauboi5.get_current_waypoint(RSHD)
        if waypoint:
            joint = waypoint['joint']
            pos = waypoint['pos']
            print("关节角度: [{0:.4f}, {1:.4f}, {2:.4f}, {3:.4f}, {4:.4f}, {5:.4f}]".format(*joint))
            print("笛卡尔坐标: X={0:.4f}, Y={1:.4f}, Z={2:.4f}".format(pos[0], pos[1], pos[2]))
            
    else:
        print("{0} 连接失败".format(get_local_time()))

    # 清理资源
    libpyauboi5.robot_shutdown(RSHD)  # 添加关闭机器人
    libpyauboi5.logout(RSHD)
    libpyauboi5.destory_context(RSHD)
    libpyauboi5.uninitialize()
    print("\n程序结束")

if __name__ == "__main__":
    main()