#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert nav cmd_vel (v, yaw_rate) to tricycle drive cmd_vel (v, steer_angle)."""
import math
import rospy
from geometry_msgs.msg import Twist


class CmdVelToTricycle:
    def __init__(self):
        rospy.init_node("cmdvel_to_tricycle")
        self.wheelbase = rospy.get_param("~wheelbase", 1.14)
        self.steer_max = rospy.get_param("~steer_max", 0.7)
        self.steer_rate_max = rospy.get_param("~steer_rate_max", 0.8)  # rad/s
        self.v_deadzone = rospy.get_param("~v_deadzone", 0.05)
        self.steer_sign = rospy.get_param("~steer_sign", -1.0)
        self.alpha = rospy.get_param("~alpha", 0.35)  # low-pass filter coefficient
        self.input_is_steer = rospy.get_param("~input_is_steer", False)
        self.last_steer = 0.0
        self.last_t = rospy.Time.now()

        self.pub = rospy.Publisher("/tricycle/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/cmd_vel_nav", Twist, self.cb, queue_size=10)

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        out = Twist()
        out.linear.x = v

        if self.input_is_steer:
            steer_cmd = msg.angular.z
        elif abs(v) < self.v_deadzone:
            steer_cmd = 0.0
        else:
            steer_cmd = math.atan(self.wheelbase * w / v)

        steer_cmd *= self.steer_sign
        now = rospy.Time.now()
        dt = max(1e-3, (now - self.last_t).to_sec())
        self.last_t = now

        # First-order low-pass + steer rate limiter for anti-oscillation.
        steer_f = self.alpha * steer_cmd + (1.0 - self.alpha) * self.last_steer
        dmax = self.steer_rate_max * dt
        steer = max(self.last_steer - dmax, min(self.last_steer + dmax, steer_f))
        steer = max(-self.steer_max, min(self.steer_max, steer))
        self.last_steer = steer

        out.angular.z = steer
        self.pub.publish(out)


if __name__ == "__main__":
    CmdVelToTricycle()
    rospy.spin()
