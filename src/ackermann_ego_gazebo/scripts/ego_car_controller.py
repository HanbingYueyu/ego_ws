#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Track EGO-Planner PositionCommand with Gazebo tricycle cmd_vel (speed + steer angle)."""
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from tf.transformations import euler_from_quaternion


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class EgoCarController:
    def __init__(self):
        rospy.init_node("ego_car_controller")
        self._last_cmd = None
        self._last_cmd_time = rospy.Time(0)
        self._yaw = 0.0
        self._v_max = rospy.get_param("~v_max", 1.0)
        self._k_w = rospy.get_param("~k_w", 1.8)
        self._steer_max = rospy.get_param("~steer_max", 1.35)
        self._cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self._cmd_cb, queue_size=10)
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._rate = rospy.Rate(50.0)

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._yaw = yaw

    def _cmd_cb(self, msg: PositionCommand):
        self._last_cmd = msg
        self._last_cmd_time = rospy.Time.now()

    def spin(self):
        twist = Twist()
        while not rospy.is_shutdown():
            cmd = self._last_cmd
            if cmd is None or (rospy.Time.now() - self._last_cmd_time).to_sec() > self._cmd_timeout:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._pub.publish(twist)
                self._rate.sleep()
                continue

            vx = cmd.velocity.x
            vy = cmd.velocity.y
            v_ref = math.sqrt(vx * vx + vy * vy)
            v_ref = max(0.0, min(self._v_max, v_ref))
            if v_ref < 0.05:
                # Near stop: still steer toward commanded yaw
                v_ref = 0.0

            yaw_des = cmd.yaw
            err = wrap_pi(yaw_des - self._yaw)
            steer = max(-self._steer_max, min(self._steer_max, self._k_w * err))

            twist.linear.x = v_ref
            twist.angular.z = steer
            self._pub.publish(twist)
            self._rate.sleep()


def main():
    EgoCarController().spin()


if __name__ == "__main__":
    main()
