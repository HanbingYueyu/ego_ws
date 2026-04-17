#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish steering/wheel joint states from /cmd_vel for Ackermann-like visualization."""

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


def clamp(value, low, high):
    return max(low, min(high, value))


class AckermannJointStateBridge:
    def __init__(self):
        rospy.init_node("ackermann_joint_state_bridge")

        self.wheelbase = float(rospy.get_param("~wheelbase", 0.5266))
        self.track = float(rospy.get_param("~track", 0.4413))
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.11))
        self.steer_max = float(rospy.get_param("~steer_max", 0.52))
        self.min_speed_for_steer = float(rospy.get_param("~min_speed_for_steer", 0.08))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.4))
        # Steering joint axes are mirrored in URDF (left ~-Z, right ~+Z),
        # so command signs need opposite compensation to produce the same turn direction.
        self.left_steer_sign = float(rospy.get_param("~left_steer_sign", -1.0))
        self.right_steer_sign = float(rospy.get_param("~right_steer_sign", 1.0))
        self.left_steer_offset = float(rospy.get_param("~left_steer_offset", 0.0))
        self.right_steer_offset = float(rospy.get_param("~right_steer_offset", 0.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 40.0))

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.last_update_time = rospy.Time.now()

        self.lb_pos = 0.0
        self.rb_pos = 0.0
        self.lf_pos = 0.0
        self.rf_pos = 0.0

        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=20)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=20)

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def compute_steer(self, v, w):
        if abs(w) < 1e-6:
            return 0.0
        # Keep steering responsive during near-zero-speed yaw commands.
        if abs(v) < self.min_speed_for_steer:
            v = math.copysign(self.min_speed_for_steer, v if abs(v) > 1e-6 else w)
        return math.atan(self.wheelbase * w / v)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_update_time).to_sec()
            if dt <= 0.0:
                dt = 1.0 / self.publish_rate
            self.last_update_time = now

            if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
                v = 0.0
                w = 0.0
            else:
                v = self.last_cmd.linear.x
                w = self.last_cmd.angular.z

            steer_center = clamp(self.compute_steer(v, w), -self.steer_max, self.steer_max)

            if abs(steer_center) < 1e-6:
                left_steer = 0.0
                right_steer = 0.0
            else:
                radius = self.wheelbase / math.tan(steer_center)
                left_steer = math.atan(self.wheelbase / (radius - self.track / 2.0))
                right_steer = math.atan(self.wheelbase / (radius + self.track / 2.0))

            left_steer = clamp(
                self.left_steer_sign * left_steer + self.left_steer_offset,
                -self.steer_max,
                self.steer_max,
            )
            right_steer = clamp(
                self.right_steer_sign * right_steer + self.right_steer_offset,
                -self.steer_max,
                self.steer_max,
            )

            wheel_omega = 0.0 if self.wheel_radius <= 1e-6 else (v / self.wheel_radius)
            self.lb_pos += wheel_omega * dt
            self.lf_pos += wheel_omega * dt
            # Right wheels use mirrored joint axes, so invert sign to keep forward roll consistent.
            self.rb_pos -= wheel_omega * dt
            self.rf_pos -= wheel_omega * dt

            msg = JointState()
            msg.header.stamp = now
            msg.name = [
                "lb_wheel_joint",
                "rb_wheel_joint",
                "left1_joint",
                "lf_wheel_joint",
                "right1_joint",
                "rf_wheel_joint",
            ]
            msg.position = [
                self.lb_pos,
                self.rb_pos,
                left_steer,
                self.lf_pos,
                right_steer,
                self.rf_pos,
            ]
            self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    AckermannJointStateBridge().spin()
