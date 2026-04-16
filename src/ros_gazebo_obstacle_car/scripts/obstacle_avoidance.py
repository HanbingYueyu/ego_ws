#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=10)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=10)
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)

        self.current_pose = None
        self.goal_pose = None

        self.cmd = Twist()
        self.max_speed = rospy.get_param("~max_speed", 0.85)
        self.min_cruise_speed = rospy.get_param("~min_cruise_speed", 0.22)
        self.rotate_speed = rospy.get_param("~rotate_speed", 1.2)
        self.slow_obstacle_dist = rospy.get_param("~slow_obstacle_dist", 1.6)
        self.stop_obstacle_dist = rospy.get_param("~stop_obstacle_dist", 0.9)
        rospy.loginfo("Goal-aware obstacle avoidance node started")

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, odom_msg):
        self.current_pose = odom_msg.pose.pose

    def goal_callback(self, goal_msg):
        self.goal_pose = goal_msg.pose
        rospy.loginfo("Received goal: (%.2f, %.2f)", self.goal_pose.position.x, self.goal_pose.position.y)

    def publish_planned_path(self):
        if self.current_pose is None or self.goal_pose is None:
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        steps = 24

        for index in range(steps + 1):
            ratio = float(index) / float(steps)
            pose_msg = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = start_x + (goal_x - start_x) * ratio
            pose_msg.pose.position.y = start_y + (goal_y - start_y) * ratio
            pose_msg.pose.position.z = 0.02
            pose_msg.pose.orientation.w = 1.0
            path_msg.poses.append(pose_msg)

        self.path_pub.publish(path_msg)

    def scan_callback(self, scan_msg):
        if not scan_msg.ranges:
            return

        center = len(scan_msg.ranges) // 2
        half_window = len(scan_msg.ranges) // 6  # about +/-30 deg
        side_window = len(scan_msg.ranges) // 8
        min_distance = scan_msg.range_max

        def valid_distance(value):
            return scan_msg.range_min < value < scan_msg.range_max

        start_index = max(0, center - half_window)
        end_index = min(len(scan_msg.ranges) - 1, center + half_window)

        for index in range(start_index, end_index + 1):
            current_distance = scan_msg.ranges[index]
            if valid_distance(current_distance) and current_distance < min_distance:
                min_distance = current_distance

        left_sum = 0.0
        left_cnt = 0
        right_sum = 0.0
        right_cnt = 0

        for index in range(center, min(len(scan_msg.ranges), center + side_window)):
            value = scan_msg.ranges[index]
            if valid_distance(value):
                left_sum += value
                left_cnt += 1

        for index in range(max(0, center - side_window), center):
            value = scan_msg.ranges[index]
            if valid_distance(value):
                right_sum += value
                right_cnt += 1

        left_clearance = left_sum / left_cnt if left_cnt > 0 else scan_msg.range_max
        right_clearance = right_sum / right_cnt if right_cnt > 0 else scan_msg.range_max
        turn_sign = 1.0 if left_clearance > right_clearance else -1.0

        if self.current_pose is None or self.goal_pose is None:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_pub.publish(self.cmd)
            return

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        goal_dist = math.hypot(dx, dy)
        self.publish_planned_path()

        if goal_dist < 0.30:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.goal_pose = None
            rospy.loginfo("Goal reached")
            self.cmd_pub.publish(self.cmd)
            return

        yaw = self.yaw_from_quat(self.current_pose.orientation)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - yaw)
        abs_yaw_error = abs(yaw_error)

        if min_distance < self.stop_obstacle_dist:
            self.cmd.linear.x = 0.08
            self.cmd.angular.z = turn_sign * self.rotate_speed
            rospy.logwarn_throttle(1.0, "Obstacle too close: %.2f m", min_distance)
        elif min_distance < self.slow_obstacle_dist:
            self.cmd.linear.x = 0.20
            self.cmd.angular.z = turn_sign * 0.55 + 0.75 * yaw_error
            rospy.loginfo_throttle(1.0, "Obstacle ahead: %.2f m", min_distance)
        elif abs_yaw_error > 1.0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = max(-self.rotate_speed, min(self.rotate_speed, 1.4 * yaw_error))
        else:
            desired_speed = max(self.min_cruise_speed, 0.55 * goal_dist)
            self.cmd.linear.x = min(self.max_speed, desired_speed)
            self.cmd.angular.z = max(-self.rotate_speed, min(self.rotate_speed, 1.35 * yaw_error))

        self.cmd_pub.publish(self.cmd)


def main():
    rospy.init_node("obstacle_avoidance_node")
    ObstacleAvoidance()
    rospy.spin()


if __name__ == "__main__":
    main()
