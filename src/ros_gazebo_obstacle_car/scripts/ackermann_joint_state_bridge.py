#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish Ackermann joint states and optionally sync steering joints to Gazebo."""

import math

import rospy
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
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
        self.left_steer_joint = str(rospy.get_param("~left_steer_joint", "left1_joint"))
        self.right_steer_joint = str(rospy.get_param("~right_steer_joint", "right1_joint"))
        self.publish_rate = float(rospy.get_param("~publish_rate", 40.0))

        self.enable_gazebo_steer_sync = bool(rospy.get_param("~sync_gazebo_steering", True))
        self.gazebo_model_name = str(rospy.get_param("~gazebo_model_name", "top_akm_dl_robot"))
        self.gazebo_urdf_param_name = str(rospy.get_param("~gazebo_urdf_param_name", "robot_description"))
        self.gazebo_service_name = str(
            rospy.get_param("~gazebo_service_name", "/gazebo/set_model_configuration")
        )
        self.gazebo_sync_rate = float(rospy.get_param("~gazebo_sync_rate", 40.0))

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.last_update_time = rospy.Time.now()
        self.last_gazebo_sync_time = rospy.Time(0)

        self.lb_pos = 0.0
        self.rb_pos = 0.0
        self.lf_pos = 0.0
        self.rf_pos = 0.0

        self.gazebo_joint_names = [self.left_steer_joint, self.right_steer_joint]
        self.set_model_config_srv = None
        if self.enable_gazebo_steer_sync:
            try:
                rospy.wait_for_service(self.gazebo_service_name, timeout=5.0)
                self.set_model_config_srv = rospy.ServiceProxy(
                    self.gazebo_service_name, SetModelConfiguration
                )
                rospy.loginfo(
                    "ackermann_joint_state_bridge: Gazebo steer sync enabled (%s)",
                    self.gazebo_service_name,
                )
            except rospy.ROSException:
                rospy.logwarn(
                    "ackermann_joint_state_bridge: service %s not available, disable Gazebo steer sync",
                    self.gazebo_service_name,
                )
                self.enable_gazebo_steer_sync = False

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

    def sync_gazebo_steering(self, now, left_steer, right_steer):
        if not self.enable_gazebo_steer_sync or self.set_model_config_srv is None:
            return

        if self.gazebo_sync_rate > 0.0:
            min_dt = 1.0 / self.gazebo_sync_rate
            if (now - self.last_gazebo_sync_time).to_sec() < min_dt:
                return

        req = SetModelConfigurationRequest()
        req.model_name = self.gazebo_model_name
        req.urdf_param_name = self.gazebo_urdf_param_name
        req.joint_names = self.gazebo_joint_names
        req.joint_positions = [left_steer, right_steer]

        try:
            resp = self.set_model_config_srv(req)
            self.last_gazebo_sync_time = now
            if not resp.success:
                rospy.logwarn_throttle(
                    2.0,
                    "ackermann_joint_state_bridge: Gazebo steer sync failed: %s",
                    resp.status_message,
                )
        except rospy.ServiceException as exc:
            self.last_gazebo_sync_time = now
            rospy.logwarn_throttle(
                2.0,
                "ackermann_joint_state_bridge: Gazebo steer sync service error: %s",
                exc,
            )

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
            self.sync_gazebo_steering(now, left_steer, right_steer)
            rate.sleep()


if __name__ == "__main__":
    AckermannJointStateBridge().spin()
