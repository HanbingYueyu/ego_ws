#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry


class WorldOdomBridge:
    def __init__(self):
        rospy.init_node("world_odom_bridge")
        self.offset_x = float(rospy.get_param("~world_offset_x", 0.0))
        self.offset_y = float(rospy.get_param("~world_offset_y", 0.0))
        self.offset_z = float(rospy.get_param("~world_offset_z", 0.0))
        odom_in = rospy.get_param("~odom_in", "mavros/local_position/odom")
        odom_out = rospy.get_param("~odom_out", "world_odom")

        self.pub = rospy.Publisher(odom_out, Odometry, queue_size=20)
        rospy.Subscriber(odom_in, Odometry, self.odom_cb, queue_size=20)

    def odom_cb(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = rospy.get_param("~world_frame", "world")
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        out.pose.pose.position.x = msg.pose.pose.position.x + self.offset_x
        out.pose.pose.position.y = msg.pose.pose.position.y + self.offset_y
        out.pose.pose.position.z = msg.pose.pose.position.z + self.offset_z
        self.pub.publish(out)


if __name__ == "__main__":
    try:
        WorldOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
