#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz marker for the car (TF is published by Gazebo tricycle plugin + world->odom)."""
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


def main():
    rospy.init_node("odom_car_marker")
    pub = rospy.Publisher("/rviz_car_marker", Marker, queue_size=1)

    def cb(msg: Odometry):
        m = Marker()
        m.header.frame_id = msg.header.frame_id if msg.header.frame_id else "odom"
        m.header.stamp = msg.header.stamp
        m.ns = "car_avatar"
        m.id = 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = msg.pose.pose
        m.scale.x = 0.9
        m.scale.y = 0.55
        m.scale.z = 0.2
        m.color.r = 0.1
        m.color.g = 0.6
        m.color.b = 0.95
        m.color.a = 1.0
        pub.publish(m)

    rospy.Subscriber("/odom", Odometry, cb, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
