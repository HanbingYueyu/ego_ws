#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

try:
    from ego_planner.msg import Bspline
except ImportError:
    Bspline = None


class FormationCollisionCloud:
    def __init__(self):
        rospy.init_node("formation_collision_cloud")
        self.lock = threading.Lock()
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.safety_radius = float(rospy.get_param("~safety_radius", 1.2))
        self.vertical_radius = float(rospy.get_param("~vertical_radius", 0.6))
        self.point_resolution = float(rospy.get_param("~point_resolution", 0.25))
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.5))
        self.traj_timeout = float(rospy.get_param("~traj_timeout", 1.5))
        self.traj_sample_horizon = float(rospy.get_param("~traj_sample_horizon", 2.5))
        self.traj_sample_dt = float(rospy.get_param("~traj_sample_dt", 0.5))
        self.traj_safety_radius = float(rospy.get_param("~traj_safety_radius", self.safety_radius))
        self.traj_vertical_radius = float(rospy.get_param("~traj_vertical_radius", self.vertical_radius))

        self.names = ["main", "uav1", "uav2", "uav3", "uav4"]
        self.odom_topics = {
            "main": rospy.get_param("~main_odom_topic", "/mavros/local_position/odom"),
            "uav1": rospy.get_param("~uav1_odom_topic", "/uav1/world_odom"),
            "uav2": rospy.get_param("~uav2_odom_topic", "/uav2/world_odom"),
            "uav3": rospy.get_param("~uav3_odom_topic", "/uav3/world_odom"),
            "uav4": rospy.get_param("~uav4_odom_topic", "/uav4/world_odom"),
        }
        self.local_cloud_topics = {
            "main": rospy.get_param("~main_local_cloud_topic", "/pcl_render_node/cloud"),
            "uav1": rospy.get_param("~uav1_local_cloud_topic", "/uav1/pcl_render_node/cloud"),
            "uav2": rospy.get_param("~uav2_local_cloud_topic", "/uav2/pcl_render_node/cloud"),
            "uav3": rospy.get_param("~uav3_local_cloud_topic", "/uav3/pcl_render_node/cloud"),
            "uav4": rospy.get_param("~uav4_local_cloud_topic", "/uav4/pcl_render_node/cloud"),
        }
        self.output_topics = {
            "main": rospy.get_param("~main_cloud_out", "/planner_cloud"),
            "uav1": rospy.get_param("~uav1_cloud_out", "/uav1/planner_cloud"),
            "uav2": rospy.get_param("~uav2_cloud_out", "/uav2/planner_cloud"),
            "uav3": rospy.get_param("~uav3_cloud_out", "/uav3/planner_cloud"),
            "uav4": rospy.get_param("~uav4_cloud_out", "/uav4/planner_cloud"),
        }
        self.bspline_topics = {
            "main": rospy.get_param("~main_bspline_topic", "/planning/bspline"),
            "uav1": rospy.get_param("~uav1_bspline_topic", "/uav1/planning/bspline"),
            "uav2": rospy.get_param("~uav2_bspline_topic", "/uav2/planning/bspline"),
            "uav3": rospy.get_param("~uav3_bspline_topic", "/uav3/planning/bspline"),
            "uav4": rospy.get_param("~uav4_bspline_topic", "/uav4/planning/bspline"),
        }

        self.positions = {}
        self.local_clouds = {}
        self.trajectories = {}
        self.pubs = {
            name: rospy.Publisher(topic, PointCloud2, queue_size=1)
            for name, topic in self.output_topics.items()
        }

        for name in self.names:
            rospy.Subscriber(
                self.odom_topics[name], Odometry,
                lambda msg, n=name: self.odom_cb(n, msg),
                queue_size=20,
            )
            rospy.Subscriber(
                self.local_cloud_topics[name], PointCloud2,
                lambda msg, n=name: self.cloud_cb(n, msg),
                queue_size=1,
            )
            if Bspline is not None:
                rospy.Subscriber(
                    self.bspline_topics[name], Bspline,
                    lambda msg, n=name: self.bspline_cb(n, msg),
                    queue_size=5,
                )

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_all)

    def odom_cb(self, name, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.positions[name] = (p.x, p.y, p.z, rospy.Time.now())

    def cloud_cb(self, name, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append((p[0], p[1], p[2]))

        with self.lock:
            self.local_clouds[name] = points

    def bspline_cb(self, name, msg):
        if not msg.pos_pts or not msg.knots or msg.order <= 0:
            return

        points = [(p.x, p.y, p.z) for p in msg.pos_pts]
        knots = list(msg.knots)
        with self.lock:
            self.trajectories[name] = {
                "order": int(msg.order),
                "start_time": msg.start_time,
                "points": points,
                "knots": knots,
                "stamp": rospy.Time.now(),
            }

    def vehicle_points(self, cx, cy, cz, radius=None, vertical_radius=None):
        radius = self.safety_radius if radius is None else radius
        vertical_radius = self.vertical_radius if vertical_radius is None else vertical_radius
        points = []
        steps_r = max(1, int(radius / self.point_resolution))
        steps_z = max(1, int((2.0 * vertical_radius) / self.point_resolution))
        angle_count = max(12, int(2.0 * math.pi * radius / self.point_resolution))

        for iz in range(steps_z + 1):
            z = cz - vertical_radius + 2.0 * vertical_radius * iz / float(steps_z)
            points.append((cx, cy, z))
            for ir in range(steps_r + 1):
                r = radius * ir / float(steps_r)
                if r < 1e-6:
                    continue
                for ia in range(angle_count):
                    a = 2.0 * math.pi * ia / float(angle_count)
                    points.append((cx + r * math.cos(a), cy + r * math.sin(a), z))

        return points

    def evaluate_bspline(self, traj, t):
        points = traj["points"]
        knots = traj["knots"]
        degree = traj["order"]
        n = len(points) - 1
        m = n + degree + 1
        if degree < 1 or len(knots) <= m or len(points) <= degree:
            return None

        u_min = knots[degree]
        u_max = knots[m - degree]
        u = min(max(t + u_min, u_min), u_max)

        k = degree
        while k + 1 < len(knots) and knots[k + 1] < u:
            k += 1
        if k - degree < 0 or k >= len(points):
            return None

        d = [list(points[k - degree + i]) for i in range(degree + 1)]
        for r in range(1, degree + 1):
            for i in range(degree, r - 1, -1):
                left = knots[i + k - degree]
                right = knots[i + 1 + k - r]
                denom = right - left
                alpha = 0.0 if abs(denom) < 1e-9 else (u - left) / denom
                d[i] = [
                    (1.0 - alpha) * d[i - 1][axis] + alpha * d[i][axis]
                    for axis in range(3)
                ]
        return tuple(d[degree])

    def sample_trajectory(self, traj, now):
        if (now - traj["stamp"]).to_sec() > self.traj_timeout:
            return []

        degree = traj["order"]
        n = len(traj["points"]) - 1
        m = n + degree + 1
        if len(traj["knots"]) <= m:
            return []

        duration = traj["knots"][m - degree] - traj["knots"][degree]
        if duration < 0.0:
            return []

        elapsed = max(0.0, (now - traj["start_time"]).to_sec())
        if elapsed > duration + self.traj_timeout:
            return []

        samples = []
        sample_count = max(1, int(self.traj_sample_horizon / self.traj_sample_dt))
        for i in range(sample_count + 1):
            t = min(duration, elapsed + i * self.traj_sample_dt)
            point = self.evaluate_bspline(traj, t)
            if point is not None:
                samples.append(point)
            if t >= duration:
                break
        return samples

    def publish_all(self, _event):
        with self.lock:
            positions = dict(self.positions)
            local_clouds = {name: list(points) for name, points in self.local_clouds.items()}
            trajectories = dict(self.trajectories)

        stamp = rospy.Time.now()
        for name in self.names:
            points = local_clouds.get(name, [])
            merged = list(points)

            for other, pos in positions.items():
                if other == name:
                    continue
                if (stamp - pos[3]).to_sec() <= self.odom_timeout:
                    merged.extend(self.vehicle_points(pos[0], pos[1], pos[2]))

            for other, traj in trajectories.items():
                if other == name:
                    continue
                for p in self.sample_trajectory(traj, stamp):
                    merged.extend(
                        self.vehicle_points(
                            p[0], p[1], p[2],
                            radius=self.traj_safety_radius,
                            vertical_radius=self.traj_vertical_radius,
                        )
                    )

            header = Header()
            header.stamp = stamp
            header.frame_id = self.frame_id
            self.pubs[name].publish(pc2.create_cloud_xyz32(header, merged))


if __name__ == "__main__":
    try:
        FormationCollisionCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
