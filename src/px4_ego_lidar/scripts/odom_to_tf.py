#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

def odom_callback(msg):
    # 1. 广播 TF 坐标树 (给雷达投影用)
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = msg.header.stamp 
    t.header.frame_id = "world"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation
    br.sendTransform(t)

    # 2. 强制在 RViz 中画一个代表无人机的“亮黄色方块”
    drone_marker = Marker()
    drone_marker.header.frame_id = "world"
    drone_marker.header.stamp = msg.header.stamp
    drone_marker.ns = "drone_avatar"
    drone_marker.id = 999
    drone_marker.type = Marker.CUBE # 方块形状
    drone_marker.action = Marker.ADD
    drone_marker.pose = msg.pose.pose # 姿态完全跟随无人机

    # 无人机大小：半米见方，厚度10厘米
    drone_marker.scale.x = 0.5
    drone_marker.scale.y = 0.5
    drone_marker.scale.z = 0.1

    # 亮黄色，极其醒目
    drone_marker.color.r = 1.0
    drone_marker.color.g = 1.0
    drone_marker.color.b = 0.0
    drone_marker.color.a = 1.0

    marker_pub.publish(drone_marker)

if __name__ == '__main__':
    rospy.init_node('px4_tf_bridge')

    # 初始化发布器
    global marker_pub
    marker_pub = rospy.Publisher('/rviz_drone_marker', Marker, queue_size=1)

    rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback, queue_size=10)
    rospy.spin()
