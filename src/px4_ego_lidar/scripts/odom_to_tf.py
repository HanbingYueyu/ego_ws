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
    t.header.frame_id = WORLD_FRAME
    t.child_frame_id = BASE_FRAME
    t.transform.translation.x = msg.pose.pose.position.x + WORLD_OFFSET_X
    t.transform.translation.y = msg.pose.pose.position.y + WORLD_OFFSET_Y
    t.transform.translation.z = msg.pose.pose.position.z + WORLD_OFFSET_Z
    t.transform.rotation = msg.pose.pose.orientation
    br.sendTransform(t)

    # 2. 强制在 RViz 中画一个代表无人机的“亮黄色方块”
    drone_marker = Marker()
    drone_marker.header.frame_id = WORLD_FRAME
    drone_marker.header.stamp = msg.header.stamp
    drone_marker.ns = "drone_avatar"
    drone_marker.id = 999
    drone_marker.type = Marker.CUBE # 方块形状
    drone_marker.action = Marker.ADD
    drone_marker.pose = msg.pose.pose # 姿态完全跟随无人机
    drone_marker.pose.position.x += WORLD_OFFSET_X
    drone_marker.pose.position.y += WORLD_OFFSET_Y
    drone_marker.pose.position.z += WORLD_OFFSET_Z

    # Raise the RViz avatar slightly so it reads clearly above the real vehicle.
    drone_marker.pose.position.z += 0.35

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
    odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
    world_frame = rospy.get_param("~world_frame", "world")
    base_frame = rospy.get_param("~base_frame", "base_link")
    marker_topic = rospy.get_param("~marker_topic", "/rviz_drone_marker")
    world_offset_x = float(rospy.get_param("~world_offset_x", 0.0))
    world_offset_y = float(rospy.get_param("~world_offset_y", 0.0))
    world_offset_z = float(rospy.get_param("~world_offset_z", 0.0))

    # 初始化发布器
    global marker_pub
    marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1)
    globals()["WORLD_FRAME"] = world_frame
    globals()["BASE_FRAME"] = base_frame
    globals()["WORLD_OFFSET_X"] = world_offset_x
    globals()["WORLD_OFFSET_Y"] = world_offset_y
    globals()["WORLD_OFFSET_Z"] = world_offset_z

    rospy.Subscriber(odom_topic, Odometry, odom_callback, queue_size=10)
    rospy.spin()
