#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def quaternion_to_yaw(quat):
    if isinstance(quat, Quaternion):
        q = [quat.x, quat.y, quat.z, quat.w]
    else:
        q = quat
    _, _, yaw = euler_from_quaternion(q)
    return yaw


class PX4FollowerController:
    def __init__(self):
        rospy.init_node("px4_follower_controller")

        self.state = State()
        self.leader_odom = None
        self.leader_odom_time = rospy.Time(0)
        self.self_odom = None
        self.self_odom_time = rospy.Time(0)
        self.front_odom = None
        self.front_odom_time = rospy.Time(0)
        self.path_samples = deque()
        self.path_length = 0.0
        self.sample_seq = 0
        self.command_s = None
        self.last_target_update_time = rospy.Time(0)

        self.trail_distance = float(rospy.get_param("~trail_distance", 1.5))
        self.offset_z = float(rospy.get_param("~offset_z", 0.0))
        self.hover_z = float(rospy.get_param("~hover_z", 1.0))
        self.odom_timeout = float(rospy.get_param("~leader_odom_timeout", 0.6))
        self.self_odom_timeout = float(rospy.get_param("~self_odom_timeout", 0.6))
        self.sample_min_dist = float(rospy.get_param("~sample_min_dist", 0.08))
        self.max_path_length = float(rospy.get_param("~max_path_length", 200.0))
        self.catchup_gain = float(rospy.get_param("~catchup_gain", 1.15))
        self.max_catchup_speed = float(rospy.get_param("~max_catchup_speed", 2.2))
        self.path_lookahead = float(rospy.get_param("~path_lookahead", 0.45))
        self.max_path_advance_speed = float(rospy.get_param("~max_path_advance_speed", 3.0))
        self.initial_x = float(rospy.get_param("~initial_x", -self.trail_distance))
        self.initial_y = float(rospy.get_param("~initial_y", 0.0))
        self.world_offset_x = float(rospy.get_param("~world_offset_x", self.initial_x))
        self.world_offset_y = float(rospy.get_param("~world_offset_y", self.initial_y))
        self.world_offset_z = float(rospy.get_param("~world_offset_z", 0.0))
        self.front_world_offset_x = float(rospy.get_param("~front_world_offset_x", 0.0))
        self.front_world_offset_y = float(rospy.get_param("~front_world_offset_y", 0.0))
        self.front_world_offset_z = float(rospy.get_param("~front_world_offset_z", 0.0))
        self.min_front_distance = float(rospy.get_param("~min_front_distance", 1.8))
        self.slow_front_distance = float(rospy.get_param("~slow_front_distance", 2.8))
        self.front_odom_timeout = float(rospy.get_param("~front_odom_timeout", 0.6))
        self.initial_yaw = float(rospy.get_param("~initial_yaw", 0.0))
        self.path_ready = False

        leader_odom_topic = rospy.get_param("~leader_odom_topic", "/mavros/local_position/odom")
        self_odom_topic = rospy.get_param("~self_odom_topic", "mavros/local_position/odom")
        front_odom_topic = rospy.get_param("~front_odom_topic", "")

        rospy.Subscriber("mavros/state", State, self.state_cb, queue_size=10)
        rospy.Subscriber(leader_odom_topic, Odometry, self.leader_odom_cb, queue_size=20)
        rospy.Subscriber(self_odom_topic, Odometry, self.self_odom_cb, queue_size=20)
        if front_odom_topic:
            rospy.Subscriber(front_odom_topic, Odometry, self.front_odom_cb, queue_size=20)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        rospy.wait_for_service("mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.target_msg = PositionTarget()
        self.target_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target_msg.type_mask = (
            PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        self.target_msg.position.x = 0.0
        self.target_msg.position.y = 0.0
        self.target_msg.position.z = self.hover_z + self.offset_z
        self.target_msg.velocity.x = 0.0
        self.target_msg.velocity.y = 0.0
        self.target_msg.velocity.z = 0.0
        self.target_msg.yaw = self.initial_yaw

    def state_cb(self, msg):
        self.state = msg

    def self_odom_cb(self, msg):
        self.self_odom = msg
        self.self_odom_time = rospy.Time.now()

    def front_odom_cb(self, msg):
        self.front_odom = msg
        self.front_odom_time = rospy.Time.now()

    def self_world_position(self):
        if self.self_odom is None:
            return None

        pose = self.self_odom.pose.pose
        return (
            pose.position.x + self.world_offset_x,
            pose.position.y + self.world_offset_y,
            pose.position.z + self.world_offset_z,
        )

    def front_world_position(self):
        if self.front_odom is None:
            return None

        if (rospy.Time.now() - self.front_odom_time).to_sec() > self.front_odom_timeout:
            return None

        pose = self.front_odom.pose.pose
        return (
            pose.position.x + self.front_world_offset_x,
            pose.position.y + self.front_world_offset_y,
            pose.position.z + self.front_world_offset_z,
        )

    def front_distance(self):
        self_pos = self.self_world_position()
        front_pos = self.front_world_position()
        if self_pos is None or front_pos is None:
            return None

        dx = front_pos[0] - self_pos[0]
        dy = front_pos[1] - self_pos[1]
        dz = front_pos[2] - self_pos[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def front_speed_scale(self):
        distance = self.front_distance()
        if distance is None:
            return 1.0

        if distance <= self.min_front_distance:
            return 0.0

        if distance >= self.slow_front_distance:
            return 1.0

        span = max(self.slow_front_distance - self.min_front_distance, 1e-3)
        return max(0.0, min(1.0, (distance - self.min_front_distance) / span))

    def append_path_sample(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = quaternion_to_yaw(pose.orientation)

        if not self.path_samples:
            virtual_start = {
                "id": self.sample_seq,
                "x": pose.position.x - self.trail_distance * math.cos(yaw),
                "y": pose.position.y - self.trail_distance * math.sin(yaw),
                "z": pose.position.z,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "yaw": yaw,
                "s": 0.0,
            }
            self.path_samples.append(virtual_start)
            self.sample_seq += 1

            self.path_length = max(self.trail_distance, 0.0)
            leader_start = {
                "id": self.sample_seq,
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "vx": twist.linear.x,
                "vy": twist.linear.y,
                "vz": twist.linear.z,
                "yaw": yaw,
                "s": self.path_length,
            }
            self.path_samples.append(leader_start)
            self.sample_seq += 1
            return

        sample = {
            "id": self.sample_seq,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "vx": twist.linear.x,
            "vy": twist.linear.y,
            "vz": twist.linear.z,
            "yaw": yaw,
            "s": self.path_length,
        }

        if self.path_samples:
            last = self.path_samples[-1]
            dx = sample["x"] - last["x"]
            dy = sample["y"] - last["y"]
            dz = sample["z"] - last["z"]
            ds = math.sqrt(dx * dx + dy * dy + dz * dz)
            if ds < self.sample_min_dist:
                self.path_samples[-1].update(
                    {
                        "vx": sample["vx"],
                        "vy": sample["vy"],
                        "vz": sample["vz"],
                        "yaw": sample["yaw"],
                    }
                )
                return
            self.path_length = last["s"] + ds
            sample["s"] = self.path_length

        self.path_samples.append(sample)
        self.sample_seq += 1

        while (
            len(self.path_samples) > 2
            and (self.path_length - self.path_samples[1]["s"]) > self.max_path_length
        ):
            self.path_samples.popleft()

    def leader_odom_cb(self, msg):
        self.leader_odom = msg
        self.leader_odom_time = rospy.Time.now()
        self.append_path_sample(msg)

    def angle_diff(self, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    def interpolate_yaw(self, yaw0, yaw1, t):
        return yaw0 + self.angle_diff(yaw1, yaw0) * t

    def interpolate_path_sample(self, target_s):
        if not self.path_samples:
            return None

        if target_s <= self.path_samples[0]["s"]:
            return dict(self.path_samples[0])

        prev = self.path_samples[0]
        for sample in self.path_samples:
            if sample["s"] < target_s:
                prev = sample
                continue

            ds = sample["s"] - prev["s"]
            if ds <= 1e-6:
                return dict(sample)

            t = (target_s - prev["s"]) / ds
            return {
                "id": sample["id"],
                "x": prev["x"] + (sample["x"] - prev["x"]) * t,
                "y": prev["y"] + (sample["y"] - prev["y"]) * t,
                "z": prev["z"] + (sample["z"] - prev["z"]) * t,
                "vx": prev["vx"] + (sample["vx"] - prev["vx"]) * t,
                "vy": prev["vy"] + (sample["vy"] - prev["vy"]) * t,
                "vz": prev["vz"] + (sample["vz"] - prev["vz"]) * t,
                "yaw": self.interpolate_yaw(prev["yaw"], sample["yaw"], t),
                "s": target_s,
            }

        return dict(self.path_samples[-1])

    def project_position_to_path_s(self, position):
        if len(self.path_samples) < 2:
            return self.path_samples[0]["s"] if self.path_samples else 0.0

        px, py, pz = position
        best_s = self.path_samples[0]["s"]
        best_dist2 = None
        prev = self.path_samples[0]

        for sample in list(self.path_samples)[1:]:
            ax, ay, az = prev["x"], prev["y"], prev["z"]
            bx, by, bz = sample["x"], sample["y"], sample["z"]
            vx, vy, vz = bx - ax, by - ay, bz - az
            wx, wy, wz = px - ax, py - ay, pz - az
            seg_len2 = vx * vx + vy * vy + vz * vz

            if seg_len2 <= 1e-9:
                t = 0.0
            else:
                t = (wx * vx + wy * vy + wz * vz) / seg_len2
                t = max(0.0, min(1.0, t))

            cx = ax + vx * t
            cy = ay + vy * t
            cz = az + vz * t
            dx, dy, dz = px - cx, py - cy, pz - cz
            dist2 = dx * dx + dy * dy + dz * dz

            if best_dist2 is None or dist2 < best_dist2:
                best_dist2 = dist2
                best_s = prev["s"] + (sample["s"] - prev["s"]) * t

            prev = sample

        return best_s

    def update_command_s(self, desired_s):
        self_pos = self.self_world_position()
        if self_pos is None:
            return desired_s

        now = rospy.Time.now()
        if self.command_s is None:
            nearest_s = self.project_position_to_path_s(self_pos)
            self.command_s = min(desired_s, nearest_s + self.path_lookahead)
            self.last_target_update_time = now
            return self.command_s

        dt = (now - self.last_target_update_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02
        self.last_target_update_time = now

        nearest_s = self.project_position_to_path_s(self_pos)
        proximity_limit_s = nearest_s + self.path_lookahead
        speed_limit_s = self.command_s + self.max_path_advance_speed * self.front_speed_scale() * dt
        next_s = min(desired_s, proximity_limit_s, speed_limit_s)

        # Keep the commanded station monotonic so the follower does not reverse
        # on the path when projection is noisy near turns.
        self.command_s = max(self.command_s, next_s)
        return self.command_s

    def set_target_world(self, target):
        target_z = max(self.hover_z, target["z"] + self.offset_z)

        self.target_msg.position.x = target["x"] - self.world_offset_x
        self.target_msg.position.y = target["y"] - self.world_offset_y
        self.target_msg.position.z = target_z - self.world_offset_z
        self.target_msg.yaw = target["yaw"]

        vx = target.get("vx", 0.0)
        vy = target.get("vy", 0.0)
        vz = target.get("vz", 0.0)

        self_pos = self.self_world_position()
        if self_pos is not None:
            vx += self.catchup_gain * (target["x"] - self_pos[0])
            vy += self.catchup_gain * (target["y"] - self_pos[1])
            vz += self.catchup_gain * (target_z - self_pos[2])

        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > self.max_catchup_speed and speed > 1e-6:
            scale = self.max_catchup_speed / speed
            vx *= scale
            vy *= scale
            vz *= scale

        front_scale = self.front_speed_scale()
        vx *= front_scale
        vy *= front_scale
        vz *= front_scale

        if front_scale <= 1e-6 and self_pos is not None:
            self.target_msg.position.x = self_pos[0] - self.world_offset_x
            self.target_msg.position.y = self_pos[1] - self.world_offset_y
            self.target_msg.position.z = self_pos[2] - self.world_offset_z

        self.target_msg.velocity.x = vx
        self.target_msg.velocity.y = vy
        self.target_msg.velocity.z = vz

    def update_target_from_leader(self):
        now = rospy.Time.now()
        if self.leader_odom is None or (now - self.leader_odom_time).to_sec() > self.odom_timeout:
            return

        if self.self_odom is None or (now - self.self_odom_time).to_sec() > self.self_odom_timeout:
            return

        if not self.path_samples:
            return

        self.path_ready = True
        desired_s = max(self.path_samples[0]["s"], self.path_length - self.trail_distance)
        target_s = self.update_command_s(desired_s)
        target = self.interpolate_path_sample(target_s)
        if target is None:
            return

        self.set_target_world(target)

    def spin(self):
        rate = rospy.Rate(50.0)

        while not rospy.is_shutdown() and not self.state.connected:
            rate.sleep()

        for _ in range(100):
            if rospy.is_shutdown():
                return
            self.target_msg.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.target_msg)
            rate.sleep()

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            self.update_target_from_leader()
            self.target_msg.header.stamp = rospy.Time.now()

            rospy.loginfo_throttle(1.0, "%s State: connected=%s, armed=%s, mode=%s", rospy.get_name(), self.state.connected, self.state.armed, self.state.mode)

            if self.state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                resp = self.set_mode_client(custom_mode="OFFBOARD")
                if resp.mode_sent:
                    rospy.loginfo_throttle(5.0, "%s OFFBOARD enabled", rospy.get_name())
                else:
                    rospy.logwarn_throttle(5.0, "%s OFFBOARD failed: %s", rospy.get_name(), resp)
                last_req = rospy.Time.now()
            elif not self.state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                resp = self.arming_client(True)
                if resp.success:
                    rospy.loginfo_throttle(5.0, "%s armed", rospy.get_name())
                else:
                    rospy.logwarn_throttle(5.0, "%s arming failed: %s", rospy.get_name(), resp)
                last_req = rospy.Time.now()

            self.local_pos_pub.publish(self.target_msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        PX4FollowerController().spin()
    except rospy.ROSInterruptException:
        pass
