#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Track EGO-Planner PositionCommand with a Gazebo tricycle cmd_vel bridge."""

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class EgoCarController:
    def __init__(self):
        rospy.init_node("ego_car_controller")
        self.last_cmd = None
        self.last_cmd_time = rospy.Time(0)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.v_max = rospy.get_param("~v_max", 1.0)
        self.k_pos = rospy.get_param("~k_pos", 0.55)
        self.k_w = rospy.get_param("~k_w", 1.8)
        self.k_w_fast_turn = rospy.get_param("~k_w_fast_turn", 2.8)
        self.use_pos_fallback = rospy.get_param("~use_pos_fallback", False)
        self.yaw_rate_sign = rospy.get_param("~yaw_rate_sign", rospy.get_param("~steer_sign", 1.0))
        self.k_yaw_damp = rospy.get_param("~k_yaw_damp", 0.35)
        self.allow_reverse_align = rospy.get_param("~allow_reverse_align", False)
        self.reverse_yaw_thresh = rospy.get_param("~reverse_yaw_thresh", 1.45)
        self.reverse_speed = rospy.get_param("~reverse_speed", 0.12)
        self.v_reverse_max = rospy.get_param("~v_reverse_max", 0.25)
        self.align_speed_cap = rospy.get_param("~align_speed_cap", 0.55)
        self.yaw_rate_max = rospy.get_param("~yaw_rate_max", rospy.get_param("~steer_max", 1.35))
        self.min_drive_speed = rospy.get_param("~min_drive_speed", 0.12)
        self.min_align_speed = rospy.get_param("~min_align_speed", 0.06)
        self.align_yaw_rate_max = rospy.get_param("~align_yaw_rate_max", rospy.get_param("~align_steer_max", 0.70))
        self.yaw_deadband = rospy.get_param("~yaw_deadband", 0.05)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.35)
        self.yaw_slow_thresh = rospy.get_param("~yaw_slow_thresh", 0.85)
        self.turn_in_place_thresh = rospy.get_param("~turn_in_place_thresh", 1.20)
        self.vel_heading_thresh = rospy.get_param("~vel_heading_thresh", 0.08)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)
        self.lookahead_time = rospy.get_param("~lookahead_time", 0.8)
        self.yaw_ref_rate_limit = rospy.get_param("~yaw_ref_rate_limit", 1.2)
        self.yaw_rate_rate_limit = rospy.get_param("~yaw_rate_rate_limit", rospy.get_param("~steer_rate_limit", 2.2))
        self.hard_brake_yaw_thresh = rospy.get_param("~hard_brake_yaw_thresh", 1.7)
        self.safety_enable = rospy.get_param("~safety_enable", True)
        self.safety_stop_dist = rospy.get_param("~safety_stop_dist", 1.05)
        self.safety_slow_dist = rospy.get_param("~safety_slow_dist", 2.2)
        self.safety_turn_rate = rospy.get_param("~safety_turn_rate", 0.95)
        self.safety_scan_timeout = rospy.get_param("~safety_scan_timeout", 0.35)
        self.safety_front_half_angle_deg = rospy.get_param("~safety_front_half_angle_deg", 35.0)
        self.safety_side_angle_deg = rospy.get_param("~safety_side_angle_deg", 75.0)
        self.debug_turn_events = rospy.get_param("~debug_turn_events", True)
        self.last_yaw_rate_cmd = 0.0
        self.last_yaw_ref = 0.0
        self.has_last_yaw_ref = False
        self.front_min_dist = float("inf")
        self.left_mean_dist = float("inf")
        self.right_mean_dist = float("inf")
        self.last_scan_time = rospy.Time(0)

        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cmd_cb, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(50.0)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_rate = msg.twist.twist.angular.z

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def scan_cb(self, msg):
        front_half = math.radians(self.safety_front_half_angle_deg)
        side_half = math.radians(self.safety_side_angle_deg)
        front_min = float("inf")
        left_sum = 0.0
        right_sum = 0.0
        left_cnt = 0
        right_cnt = 0

        for i, rng in enumerate(msg.ranges):
            if math.isnan(rng) or math.isinf(rng):
                continue
            if rng < msg.range_min or rng > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment

            if abs(ang) <= front_half:
                front_min = min(front_min, rng)

            if 0.0 < ang <= side_half:
                left_sum += rng
                left_cnt += 1
            elif -side_half <= ang < 0.0:
                right_sum += rng
                right_cnt += 1

        self.front_min_dist = front_min
        self.left_mean_dist = left_sum / left_cnt if left_cnt > 0 else float("inf")
        self.right_mean_dist = right_sum / right_cnt if right_cnt > 0 else float("inf")
        self.last_scan_time = rospy.Time.now()

    def spin(self):
        twist = Twist()
        dt = 1.0 / 50.0
        while not rospy.is_shutdown():
            cmd = self.last_cmd
            if cmd is None or (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.last_yaw_rate_cmd = 0.0
                self.pub.publish(twist)
                self.rate.sleep()
                continue

            vx = cmd.velocity.x
            vy = cmd.velocity.y
            dx = cmd.position.x - self.x
            dy = cmd.position.y - self.y
            dist = math.sqrt(dx * dx + dy * dy)

            v_ff = math.sqrt(vx * vx + vy * vy)
            if v_ff > 0.05:
                v_ref = v_ff
            else:
                # Keep planner authority by default. Optional fallback can be enabled
                # only when users want aggressive target-seeking at trajectory gaps.
                v_ref = 0.0
                if self.use_pos_fallback and dist > self.goal_tolerance:
                    v_ref = self.k_pos * dist
                    v_ref = max(v_ref, self.min_drive_speed)

            if dist < self.goal_tolerance and v_ff < 0.05:
                v_ref = 0.0

            # Use a lookahead target to make heading robust to local trajectory jitter.
            tx = cmd.position.x
            ty = cmd.position.y
            if v_ff > self.vel_heading_thresh:
                tx += self.lookahead_time * vx
                ty += self.lookahead_time * vy

            lx = tx - self.x
            ly = ty - self.y
            if math.hypot(lx, ly) > 0.03:
                yaw_ref = math.atan2(ly, lx)
            else:
                yaw_ref = cmd.yaw if math.isfinite(cmd.yaw) else self.yaw

            yaw_ref_limited = False
            if self.has_last_yaw_ref:
                dyaw_ref = wrap_pi(yaw_ref - self.last_yaw_ref)
                max_yaw_ref_step = self.yaw_ref_rate_limit * dt
                raw_dyaw_ref = dyaw_ref
                dyaw_ref = max(-max_yaw_ref_step, min(max_yaw_ref_step, dyaw_ref))
                yaw_ref_limited = abs(raw_dyaw_ref - dyaw_ref) > 1e-6
                yaw_ref = wrap_pi(self.last_yaw_ref + dyaw_ref)

            self.last_yaw_ref = yaw_ref
            self.has_last_yaw_ref = True

            yaw_error = wrap_pi(yaw_ref - self.yaw)
            abs_yaw_error = abs(yaw_error)
            k_turn = self.k_w_fast_turn if abs_yaw_error > self.yaw_slow_thresh else self.k_w
            yaw_rate_cmd = self.yaw_rate_sign * (k_turn * yaw_error - self.k_yaw_damp * self.yaw_rate)
            yaw_rate_limit = self.yaw_rate_max
            turn_phase = "track"

            # Large heading error: move slowly while steering, avoid stationary steering lock.
            if abs_yaw_error > self.hard_brake_yaw_thresh:
                if self.allow_reverse_align and abs_yaw_error > self.reverse_yaw_thresh:
                    turn_phase = "align_reverse"
                    v_ref = -self.reverse_speed
                else:
                    turn_phase = "align_hard"
                    v_ref = max(self.min_align_speed, min(v_ref, self.align_speed_cap * 0.75))
                yaw_rate_limit = self.align_yaw_rate_max
            elif abs_yaw_error > self.turn_in_place_thresh:
                turn_phase = "align"
                v_ref = max(self.min_align_speed, min(v_ref * 0.65, self.align_speed_cap))
                yaw_rate_limit = self.align_yaw_rate_max
            elif abs_yaw_error > self.yaw_slow_thresh:
                turn_phase = "slow"
                v_ref = v_ref * 0.35

            if abs_yaw_error < self.yaw_deadband:
                yaw_rate_cmd = 0.0

            yaw_rate_cmd = max(-yaw_rate_limit, min(yaw_rate_limit, yaw_rate_cmd))

            # Safety layer for nonholonomic ground robot: cap speed by frontal clearance.
            # This prevents hard collisions when global/local planner produces too-tight paths.
            safety_phase = "none"
            if self.safety_enable and (rospy.Time.now() - self.last_scan_time).to_sec() <= self.safety_scan_timeout:
                front = self.front_min_dist
                if math.isfinite(front):
                    if front < self.safety_stop_dist:
                        safety_phase = "stop_turn"
                        turn_sign = 1.0 if self.left_mean_dist >= self.right_mean_dist else -1.0
                        v_ref = min(v_ref, 0.0)
                        yaw_rate_cmd = turn_sign * max(abs(yaw_rate_cmd), self.safety_turn_rate)
                    elif front < self.safety_slow_dist:
                        safety_phase = "slow"
                        ratio = (front - self.safety_stop_dist) / max(1e-3, (self.safety_slow_dist - self.safety_stop_dist))
                        ratio = max(0.0, min(1.0, ratio))
                        v_ref = min(v_ref, v_ref * ratio)
                        if ratio < 0.35 and abs(yaw_rate_cmd) < 0.25:
                            turn_sign = 1.0 if self.left_mean_dist >= self.right_mean_dist else -1.0
                            yaw_rate_cmd = turn_sign * 0.35

            if self.debug_turn_events and (yaw_ref_limited or abs_yaw_error > self.turn_in_place_thresh):
                align_metric = yaw_error * self.yaw_rate
                rospy.logwarn_throttle(
                    0.5,
                    "turn_event phase=%s safety=%s front=%.2f sign=%.1f yaw=%.2f yaw_ref=%.2f err=%.2f yaw_rate=%.2f errYawRate=%.3f dist=%.2f v_ff=%.2f v_cmd=%.2f w=%.2f lim=%s",
                    turn_phase,
                    safety_phase,
                    self.front_min_dist if math.isfinite(self.front_min_dist) else -1.0,
                    self.yaw_rate_sign,
                    self.yaw,
                    yaw_ref,
                    yaw_error,
                    self.yaw_rate,
                    align_metric,
                    dist,
                    v_ff,
                    v_ref,
                    yaw_rate_cmd,
                    str(yaw_ref_limited),
                )

            v_min = -self.v_reverse_max if self.allow_reverse_align else 0.0
            v_ref = max(v_min, min(self.v_max, v_ref))

            max_yaw_rate_step = self.yaw_rate_rate_limit * dt
            yaw_rate_delta = yaw_rate_cmd - self.last_yaw_rate_cmd
            yaw_rate_delta = max(-max_yaw_rate_step, min(max_yaw_rate_step, yaw_rate_delta))
            yaw_rate_cmd = self.last_yaw_rate_cmd + yaw_rate_delta
            self.last_yaw_rate_cmd = yaw_rate_cmd

            twist.linear.x = v_ref
            twist.angular.z = yaw_rate_cmd
            self.pub.publish(twist)
            self.rate.sleep()


def main():
    EgoCarController().spin()


if __name__ == "__main__":
    main()
