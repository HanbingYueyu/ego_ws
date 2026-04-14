#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand

current_state = State()
target_msg = PositionTarget()

# 回调函数：记录飞控当前状态
def state_cb(msg):
    global current_state
    current_state = msg

# 回调函数：接收 EGO-Planner 的轨迹指令并“翻译”
def cmd_cb(msg):
    global target_msg
    target_msg.header.stamp = rospy.Time.now()
    target_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    # 忽略速度和加速度，只使用纯位置控制
    target_msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                           PositionTarget.IGNORE_YAW_RATE
    target_msg.position.x = msg.position.x
    target_msg.position.y = msg.position.y
    target_msg.position.z = msg.position.z
    target_msg.yaw = msg.yaw

def main():
    rospy.init_node('px4_traj_controller')

    # 订阅飞控状态和 EGO 算法指令，发布底层控制指令
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("planning/pos_cmd", PositionCommand, cmd_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)

    # 注册服务：解锁和切换飞行模式
    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(50.0) # 控制频率 50Hz

    # 等待底层飞控连接
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    rospy.loginfo("PX4 Connected! Initializing hover point...")

    # 设定默认悬停点 (让无人机先起飞到 1米 的高度等待指令)
    global target_msg
    target_msg.header.stamp = rospy.Time.now()
    target_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    target_msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                           PositionTarget.IGNORE_YAW_RATE
    target_msg.position.x = 0.0
    target_msg.position.y = 0.0
    target_msg.position.z = 1.0
    target_msg.yaw = 0.0

    # 在请求 Offboard 模式前，必须先发一阵子目标点 (这是 PX4 飞控的安全强制要求)
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(target_msg)
        rate.sleep()

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        # 状态机：自动切 Offboard 模式并解锁起飞
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("OFFBOARD enabled! Drone taking off...")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client(True).success:
                    rospy.loginfo("Vehicle armed!")
                last_req = rospy.Time.now()

        # 以 50Hz 频率持续发送坐标
        local_pos_pub.publish(target_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
