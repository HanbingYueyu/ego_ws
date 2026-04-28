#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


class StartLineMission:
    def __init__(self):
        rospy.init_node("start_line_mission")

        self.enabled = bool(rospy.get_param("~enabled", True))
        self.uav_goal_topic = rospy.get_param("~uav_goal_topic", "/move_base_simple/goal")
        self.car_goal_topic = rospy.get_param("~car_goal_topic", "/car/move_base_simple/goal")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.goal_x = float(rospy.get_param("~goal_x", 20.0))
        self.goal_y = float(rospy.get_param("~goal_y", 0.0))
        self.uav_goal_z = float(rospy.get_param("~uav_goal_z", 1.0))
        self.car_goal_z = float(rospy.get_param("~car_goal_z", 0.0))
        self.min_takeoff_z = float(rospy.get_param("~min_takeoff_z", 0.75))
        self.ready_hold_time = float(rospy.get_param("~ready_hold_time", 2.0))
        self.max_wait_time = float(rospy.get_param("~max_wait_time", 90.0))
        self.publish_count = int(rospy.get_param("~publish_count", 5))
        self.publish_interval = float(rospy.get_param("~publish_interval", 0.4))

        self.names = ["main", "uav1", "uav2", "uav3", "uav4"]
        self.state_topics = {
            "main": rospy.get_param("~main_state_topic", "/mavros/state"),
            "uav1": rospy.get_param("~uav1_state_topic", "/uav1/mavros/state"),
            "uav2": rospy.get_param("~uav2_state_topic", "/uav2/mavros/state"),
            "uav3": rospy.get_param("~uav3_state_topic", "/uav3/mavros/state"),
            "uav4": rospy.get_param("~uav4_state_topic", "/uav4/mavros/state"),
        }
        self.odom_topics = {
            "main": rospy.get_param("~main_odom_topic", "/world_odom"),
            "uav1": rospy.get_param("~uav1_odom_topic", "/uav1/world_odom"),
            "uav2": rospy.get_param("~uav2_odom_topic", "/uav2/world_odom"),
            "uav3": rospy.get_param("~uav3_odom_topic", "/uav3/world_odom"),
            "uav4": rospy.get_param("~uav4_odom_topic", "/uav4/world_odom"),
        }
        self.car_odom_topic = rospy.get_param("~car_odom_topic", "/odom")

        self.states = {}
        self.odoms = {}
        self.car_odom_ready = False

        self.uav_goal_pub = rospy.Publisher(self.uav_goal_topic, PoseStamped, queue_size=1, latch=True)
        self.car_goal_pub = rospy.Publisher(self.car_goal_topic, PoseStamped, queue_size=1, latch=True)

        for name in self.names:
            rospy.Subscriber(
                self.state_topics[name], State,
                lambda msg, n=name: self.state_cb(n, msg),
                queue_size=10,
            )
            rospy.Subscriber(
                self.odom_topics[name], Odometry,
                lambda msg, n=name: self.odom_cb(n, msg),
                queue_size=10,
            )
        rospy.Subscriber(self.car_odom_topic, Odometry, self.car_odom_cb, queue_size=10)

    def state_cb(self, name, msg):
        self.states[name] = msg

    def odom_cb(self, name, msg):
        self.odoms[name] = msg

    def car_odom_cb(self, _msg):
        self.car_odom_ready = True

    def all_uavs_ready(self):
        for name in self.names:
            state = self.states.get(name)
            odom = self.odoms.get(name)
            if state is None or odom is None:
                return False
            if not state.connected or not state.armed or state.mode != "OFFBOARD":
                return False
            if odom.pose.pose.position.z < self.min_takeoff_z:
                return False
        return True

    def make_goal(self, z):
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        return goal

    def publish_goals(self):
        for _ in range(max(1, self.publish_count)):
            self.uav_goal_pub.publish(self.make_goal(self.uav_goal_z))
            self.car_goal_pub.publish(self.make_goal(self.car_goal_z))
            rospy.sleep(self.publish_interval)

    def run(self):
        if not self.enabled:
            rospy.loginfo("start_line_mission disabled")
            return

        start_time = rospy.Time.now()
        ready_since = None
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            if self.all_uavs_ready() and self.car_odom_ready:
                if ready_since is None:
                    ready_since = rospy.Time.now()
                if (rospy.Time.now() - ready_since).to_sec() >= self.ready_hold_time:
                    rospy.loginfo(
                        "all vehicles initialized, sending field-crossing goals to x=%.2f y=%.2f",
                        self.goal_x, self.goal_y,
                    )
                    self.publish_goals()
                    return
            else:
                ready_since = None

            if self.max_wait_time > 0.0 and (rospy.Time.now() - start_time).to_sec() > self.max_wait_time:
                rospy.logwarn("start_line_mission timed out before all vehicles were ready")
                return

            rate.sleep()


if __name__ == "__main__":
    try:
        StartLineMission().run()
    except rospy.ROSInterruptException:
        pass
