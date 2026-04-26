#! /usr/bin/env python
# coding=utf-8
import time

import libpyauboi5

RS_SUCC = 0

print(__name__)


# pdb.set_trace();

# 获取系统当前时间
def get_local_time():
    return time.strftime("%b %d %Y %H:%M:%S:", time.localtime(time.time()))


def get_local_mes_time():
    ct = time.time()
    data_msecs = (ct-int(ct))*1000
    time_msecs = str("%03d" % data_msecs)
    return time_msecs



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

        # #启动参数
        #tool_dynamics = {"position": (0, 0, 0),
        #                  "payload": 1.0,
        #                  "inertia": (0, 0, 0, 0, 0, 0)}
        
        # #碰撞等级
        #collision = 6
        #

	
        #print("{0}{1}-------startup start".format(get_local_time(),get_local_mes_time()))

        #libpyauboi5.robot_startup(RSHD, collision, tool_dynamics)

        #print("{0}{1}-------startup end".format(get_local_time(),get_local_mes_time()))
	
        # 设置机械臂事件回调函数
        #libpyauboi5.setcallback_robot_event(RSHD, robot_event)

        # 关节实时信息
        joint_status = libpyauboi5.get_joint_status(RSHD)
        print("joint_status={0}".format(joint_status))

        # 获取关节当前位置
        waypoint = libpyauboi5.get_current_waypoint(RSHD)
        print("currrent waypoint={0}".format(waypoint))

        # 关节实时信息
        joint_status = libpyauboi5.get_joint_status(RSHD)
        print("joint_status={0}".format(joint_status))

        # 获取关节当前位置
        waypoint = libpyauboi5.get_current_waypoint(RSHD)
        print("currrent waypoint={0}".format(waypoint))

        # 获取IO配置
        ControllerDI = 0
        iotype = ControllerDI
        io_config = libpyauboi5.get_board_io_config(RSHD, iotype)
        #print("io_config={0}".format(io_config))

        # 初始化机械臂控制全局属性
        #libpyauboi5.init_profile(RSHD)
        libpyauboi5.init_global_move_profile(RSHD)
        #pyauboi5_init_global_move_profile

        joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
        result = libpyauboi5.move_joint(RSHD, joint_radian)

        print("机械臂轴动1")

        time.sleep(0.2)

        joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
        result = libpyauboi5.move_joint(RSHD, joint_radian)

        print("机械臂轴动2")

        time.sleep(0.2)

        

        #print("-----------------------退出登陆----------------------------")
        # 注销登陆
        #libpyauboi5.logout(RSHD)
        #print("logout.")

    else:  # 登陆失败
        print("login {0}:{1} failed.".format(ip_addr, port))  # 释放robot service控制上下文

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