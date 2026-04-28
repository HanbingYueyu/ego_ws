#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert LaserScan to PointCloud2 in world frame for RViz."""

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2 as pc2


class ScanCloudWorld:
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "world")
        self.scan_in = rospy.get_param("~scan_in", "/scan")
        self.cloud_out = rospy.get_param("~cloud_out", "/scan_cloud_world")
        self.max_points = int(rospy.get_param("~max_points", 720))
        self.use_inf_points = bool(rospy.get_param("~use_inf_points", False))
        self.range_max_margin = float(rospy.get_param("~range_max_margin", 0.08))
        self.ignore_box_enable = bool(rospy.get_param("~ignore_box_enable", False))
        self.ignore_box_center_x = float(rospy.get_param("~ignore_box_center_x", 0.0))
        self.ignore_box_center_y = float(rospy.get_param("~ignore_box_center_y", 0.0))
        self.ignore_box_center_z = float(rospy.get_param("~ignore_box_center_z", 1.0))
        self.ignore_box_size_x = float(rospy.get_param("~ignore_box_size_x", 1.2))
        self.ignore_box_size_y = float(rospy.get_param("~ignore_box_size_y", 1.2))
        self.ignore_box_size_z = float(rospy.get_param("~ignore_box_size_z", 1.2))
        self.ignore_odom_topic = rospy.get_param("~ignore_odom_topic", "")
        self.ignore_odom_timeout = float(rospy.get_param("~ignore_odom_timeout", 0.5))
        self.last_ignore_odom_time = rospy.Time(0)
        self.tf_listener = tf.TransformListener()
        self.publisher = rospy.Publisher(self.cloud_out, PointCloud2, queue_size=2)
        rospy.Subscriber(self.scan_in, LaserScan, self.scan_callback, queue_size=2)
        if self.ignore_odom_topic:
            rospy.Subscriber(self.ignore_odom_topic, Odometry, self.ignore_odom_callback, queue_size=5)
        rospy.loginfo("laserscan_cloud_world: %s -> %s", self.scan_in, self.cloud_out)

    def ignore_odom_callback(self, msg):
        self.ignore_box_center_x = msg.pose.pose.position.x
        self.ignore_box_center_y = msg.pose.pose.position.y
        self.ignore_box_center_z = msg.pose.pose.position.z
        self.last_ignore_odom_time = rospy.Time.now()

    def scan_callback(self, scan_msg):
        source_frame = scan_msg.header.frame_id
        if not source_frame:
            return

        stamp = scan_msg.header.stamp if scan_msg.header.stamp != rospy.Time() else rospy.Time(0)
        try:
            self.tf_listener.waitForTransform(
                self.target_frame, source_frame, stamp, rospy.Duration(0.2)
            )
            translation, rotation = self.tf_listener.lookupTransform(
                self.target_frame, source_frame, stamp
            )
            transform = tf.transformations.quaternion_matrix(rotation)
            transform[0, 3] = translation[0]
            transform[1, 3] = translation[1]
            transform[2, 3] = translation[2]
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "scan->cloud tf %s->%s failed: %s", source_frame, self.target_frame, exc)
            return

        world_points = []
        step = max(1, len(scan_msg.ranges) // self.max_points)
        for index in range(0, len(scan_msg.ranges), step):
            distance = scan_msg.ranges[index]
            if math.isnan(distance) or distance < scan_msg.range_min:
                continue
            if math.isinf(distance) or distance > scan_msg.range_max:
                if self.use_inf_points:
                    distance = scan_msg.range_max
                else:
                    continue
            elif (not self.use_inf_points) and distance >= (scan_msg.range_max - self.range_max_margin):
                continue
            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0
            mapped = transform.dot([x, y, z, 1.0])
            ignore_box_active = self.ignore_box_enable
            if ignore_box_active and self.ignore_odom_topic:
                ignore_box_active = (
                    self.last_ignore_odom_time != rospy.Time(0)
                    and (rospy.Time.now() - self.last_ignore_odom_time).to_sec() <= self.ignore_odom_timeout
                )
            if ignore_box_active:
                if (
                    abs(mapped[0] - self.ignore_box_center_x) <= self.ignore_box_size_x * 0.5
                    and abs(mapped[1] - self.ignore_box_center_y) <= self.ignore_box_size_y * 0.5
                    and abs(mapped[2] - self.ignore_box_center_z) <= self.ignore_box_size_z * 0.5
                ):
                    continue
            world_points.append([mapped[0], mapped[1], mapped[2]])

        if not world_points:
            return

        cloud_world = pc2.create_cloud_xyz32(scan_msg.header, world_points)
        cloud_world.header.frame_id = self.target_frame
        cloud_world.header.stamp = scan_msg.header.stamp
        self.publisher.publish(cloud_world)


def main():
    rospy.init_node("laserscan_cloud_world")
    ScanCloudWorld()
    rospy.spin()


if __name__ == "__main__":
    main()
