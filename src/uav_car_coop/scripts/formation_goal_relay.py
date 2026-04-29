#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class FormationGoalRelay:
    def __init__(self):
        rospy.init_node("formation_goal_relay")
        self.input_topic = rospy.get_param("~input_goal_topic", "/move_base_simple/goal")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.separation_enable = bool(rospy.get_param("~separation_enable", True))
        self.separation_enter_distance = float(rospy.get_param("~separation_enter_distance", 3.2))
        self.separation_exit_distance = float(rospy.get_param("~separation_exit_distance", 3.8))
        self.separation_gain = float(rospy.get_param("~separation_gain", 1.15))
        self.separation_max_bias = float(rospy.get_param("~separation_max_bias", 2.4))
        self.separation_publish_rate = float(rospy.get_param("~separation_publish_rate", 3.0))
        self.nominal_publish_rate = float(rospy.get_param("~nominal_publish_rate", 0.0))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.2))

        self.last_goal = None
        self.positions = {}
        self.separation_active = False

        self.entities = {
            "main": {
                "offset": (0.0, 0.0, 0.0),
                "odom_topic": rospy.get_param("~main_odom_topic", "/mavros/local_position/odom"),
            }
        }

        self.followers = []
        for idx in range(1, 5):
            name = "uav%d" % idx
            topic = rospy.get_param("~uav%d_goal_topic" % idx, "/uav%d/move_base_simple/goal" % idx)
            offset_x = float(rospy.get_param("~uav%d_offset_x" % idx, 0.0))
            offset_y = float(rospy.get_param("~uav%d_offset_y" % idx, 0.0))
            offset_z = float(rospy.get_param("~uav%d_offset_z" % idx, 0.0))
            odom_topic = rospy.get_param("~uav%d_odom_topic" % idx, "/uav%d/world_odom" % idx)
            pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
            follower = {
                "name": name,
                "pub": pub,
                "offset": (offset_x, offset_y, offset_z),
                "odom_topic": odom_topic,
            }
            self.followers.append(follower)
            self.entities[name] = follower

        rospy.Subscriber(self.input_topic, PoseStamped, self.goal_cb, queue_size=5)
        for name, entity in self.entities.items():
            rospy.Subscriber(
                entity["odom_topic"], Odometry,
                lambda msg, n=name: self.odom_cb(n, msg),
                queue_size=20,
            )

        if self.separation_enable:
            period = 1.0 / max(0.1, self.separation_publish_rate)
            rospy.Timer(rospy.Duration(period), self.separation_timer_cb)
        if self.nominal_publish_rate > 0.0:
            period = 1.0 / max(0.05, self.nominal_publish_rate)
            rospy.Timer(rospy.Duration(period), self.nominal_timer_cb)

    def goal_cb(self, msg):
        self.last_goal = copy.deepcopy(msg)
        self.publish_followers()

    def odom_cb(self, name, msg):
        p = msg.pose.pose.position
        self.positions[name] = (p.x, p.y, p.z, rospy.Time.now())

    def make_goal(self, follower, bias):
        goal = PoseStamped()
        goal.header = copy.deepcopy(self.last_goal.header)
        if not goal.header.frame_id:
            goal.header.frame_id = self.frame_id
        goal.header.stamp = rospy.Time.now()
        goal.pose = copy.deepcopy(self.last_goal.pose)

        offset_x, offset_y, offset_z = follower["offset"]
        goal.pose.position.x += offset_x + bias[0]
        goal.pose.position.y += offset_y + bias[1]
        goal.pose.position.z += offset_z + bias[2]
        return goal

    def publish_followers(self, biases=None):
        if self.last_goal is None:
            return
        if biases is None:
            biases = {}

        for follower in self.followers:
            bias = biases.get(follower["name"], (0.0, 0.0, 0.0))
            follower["pub"].publish(self.make_goal(follower, bias))

    def valid_positions(self):
        now = rospy.Time.now()
        valid = {}
        for name, pos in self.positions.items():
            if (now - pos[3]).to_sec() <= self.odom_timeout:
                valid[name] = pos
        return valid

    def fallback_direction(self, name, other):
        own_offset = self.entities[name]["offset"]
        other_offset = self.entities[other]["offset"]
        dx = own_offset[0] - other_offset[0]
        dy = own_offset[1] - other_offset[1]
        norm = math.hypot(dx, dy)
        if norm > 1e-3:
            return dx / norm, dy / norm

        order = {"uav1": -2.0, "uav2": -1.0, "main": 0.0, "uav3": 1.0, "uav4": 2.0}
        sign = 1.0 if order.get(name, 0.0) >= order.get(other, 0.0) else -1.0
        return 0.0, sign

    def compute_separation_biases(self, positions):
        biases = {f["name"]: [0.0, 0.0, 0.0] for f in self.followers}
        active_needed = False
        names = list(self.entities.keys())

        for i, name in enumerate(names):
            if name not in positions:
                continue
            for other in names[i + 1:]:
                if other not in positions:
                    continue

                px, py, _pz, _stamp = positions[name]
                ox, oy, _oz, _other_stamp = positions[other]
                dx = px - ox
                dy = py - oy
                dist = math.hypot(dx, dy)

                if dist < self.separation_enter_distance:
                    active_needed = True

                if not self.separation_active and dist >= self.separation_enter_distance:
                    continue
                if self.separation_active and dist >= self.separation_exit_distance:
                    continue

                if dist < 0.05:
                    ux, uy = self.fallback_direction(name, other)
                else:
                    ux, uy = dx / dist, dy / dist

                push = max(0.0, self.separation_exit_distance - dist) * self.separation_gain

                if name in biases:
                    biases[name][0] += ux * push
                    biases[name][1] += uy * push
                if other in biases:
                    biases[other][0] -= ux * push
                    biases[other][1] -= uy * push

        clipped = {}
        for name, bias in biases.items():
            norm = math.hypot(bias[0], bias[1])
            if norm > self.separation_max_bias:
                scale = self.separation_max_bias / norm
                bias[0] *= scale
                bias[1] *= scale
            clipped[name] = tuple(bias)

        return clipped, active_needed

    def separation_timer_cb(self, _event):
        if self.last_goal is None:
            return

        positions = self.valid_positions()
        biases, active_needed = self.compute_separation_biases(positions)

        was_active = self.separation_active
        if active_needed:
            self.separation_active = True
        elif self.separation_active:
            any_inside_exit = False
            values = list(positions.items())
            for idx, (name, pos) in enumerate(values):
                for other, other_pos in values[idx + 1:]:
                    if name not in self.entities or other not in self.entities:
                        continue
                    if math.hypot(pos[0] - other_pos[0], pos[1] - other_pos[1]) < self.separation_exit_distance:
                        any_inside_exit = True
                        break
                if any_inside_exit:
                    break
            self.separation_active = any_inside_exit

        if self.separation_active:
            self.publish_followers(biases)
            rospy.logwarn_throttle(1.0, "formation separation active: publishing temporary spread-out goals")
        elif was_active:
            self.publish_followers()
            rospy.loginfo("formation separation clear: restored nominal formation goals")

    def nominal_timer_cb(self, _event):
        if self.last_goal is None or self.separation_active:
            return
        self.publish_followers()


if __name__ == "__main__":
    try:
        FormationGoalRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
