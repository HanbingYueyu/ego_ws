#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def odom_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    # 核心魔法：直接将 EGO 的 world 和 飞机的 base_link 连起来
    t.header.frame_id = "world"
    t.child_frame_id = "base_link"

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('px4_tf_bridge')
    rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback)
    rospy.spin()
