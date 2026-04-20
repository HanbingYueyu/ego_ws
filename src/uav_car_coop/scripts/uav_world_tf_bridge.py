#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class UavWorldTfBridge:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.drone_frame = rospy.get_param("~drone_frame", "uav_base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "velodyne_link")
        self.lidar_z_offset = rospy.get_param("~lidar_z_offset", 0.2)
        self.publish_marker = rospy.get_param("~publish_marker", True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.marker_pub = rospy.Publisher("/rviz_drone_marker", Marker, queue_size=1)

        odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=20)

    def odom_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

        t_drone = TransformStamped()
        t_drone.header.stamp = stamp
        t_drone.header.frame_id = self.world_frame
        t_drone.child_frame_id = self.drone_frame
        t_drone.transform.translation.x = msg.pose.pose.position.x
        t_drone.transform.translation.y = msg.pose.pose.position.y
        t_drone.transform.translation.z = msg.pose.pose.position.z
        t_drone.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t_drone)

        t_lidar = TransformStamped()
        t_lidar.header.stamp = stamp
        t_lidar.header.frame_id = self.world_frame
        t_lidar.child_frame_id = self.lidar_frame
        t_lidar.transform.translation.x = msg.pose.pose.position.x
        t_lidar.transform.translation.y = msg.pose.pose.position.y
        t_lidar.transform.translation.z = msg.pose.pose.position.z + self.lidar_z_offset
        t_lidar.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t_lidar)

        if self.publish_marker:
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = stamp
            marker.ns = "uav_avatar"
            marker.id = 999
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = msg.pose.pose
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)


def main():
    rospy.init_node("uav_world_tf_bridge")
    UavWorldTfBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
