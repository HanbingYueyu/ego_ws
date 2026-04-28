#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class FormationTrajectoryViz:
    def __init__(self):
        rospy.init_node("formation_trajectory_viz")

        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_rate = float(rospy.get_param("~publish_rate", 8.0))
        self.sample_dt = float(rospy.get_param("~sample_dt", 0.15))
        self.traj_timeout = float(rospy.get_param("~traj_timeout", 2.0))
        self.line_width = float(rospy.get_param("~line_width", 0.09))

        self.names = ["main", "uav1", "uav2", "uav3", "uav4"]
        self.topics = {
            "main": rospy.get_param("~main_bspline_topic", "/planning/bspline"),
            "uav1": rospy.get_param("~uav1_bspline_topic", "/uav1/planning/bspline"),
            "uav2": rospy.get_param("~uav2_bspline_topic", "/uav2/planning/bspline"),
            "uav3": rospy.get_param("~uav3_bspline_topic", "/uav3/planning/bspline"),
            "uav4": rospy.get_param("~uav4_bspline_topic", "/uav4/planning/bspline"),
        }
        self.colors = {
            "main": (0.0, 0.65, 1.0, 1.0),
            "uav1": (1.0, 0.48, 0.0, 1.0),
            "uav2": (0.0, 0.86, 0.45, 1.0),
            "uav3": (0.78, 0.28, 1.0, 1.0),
            "uav4": (1.0, 0.18, 0.30, 1.0),
        }

        self.trajectories = {}
        self.pub = rospy.Publisher("/formation/trajectory_markers", MarkerArray, queue_size=1)

        for name in self.names:
            rospy.Subscriber(
                self.topics[name], Bspline,
                lambda msg, n=name: self.bspline_cb(n, msg),
                queue_size=5,
            )

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish)

    def bspline_cb(self, name, msg):
        if not msg.pos_pts or not msg.knots or msg.order <= 0:
            return
        self.trajectories[name] = {
            "order": int(msg.order),
            "start_time": msg.start_time,
            "points": [(p.x, p.y, p.z) for p in msg.pos_pts],
            "knots": list(msg.knots),
            "stamp": rospy.Time.now(),
        }

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
        return d[degree]

    def make_marker(self, name, marker_id, now):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = now
        marker.ns = "formation_traj"
        marker.id = marker_id

        traj = self.trajectories.get(name)
        if traj is None or (now - traj["stamp"]).to_sec() > self.traj_timeout:
            marker.action = Marker.DELETE
            return marker

        degree = traj["order"]
        n = len(traj["points"]) - 1
        m = n + degree + 1
        if len(traj["knots"]) <= m:
            marker.action = Marker.DELETE
            return marker

        duration = traj["knots"][m - degree] - traj["knots"][degree]
        if duration <= 0.0:
            marker.action = Marker.DELETE
            return marker

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        color = self.colors[name]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        elapsed = max(0.0, (now - traj["start_time"]).to_sec())
        t = min(elapsed, duration)
        while t <= duration + 1e-6:
            p = self.evaluate_bspline(traj, t)
            if p is not None:
                pt = Point()
                pt.x, pt.y, pt.z = p[0], p[1], p[2]
                marker.points.append(pt)
            t += self.sample_dt

        if len(marker.points) < 2:
            marker.action = Marker.DELETE
        return marker

    def publish(self, _event):
        now = rospy.Time.now()
        msg = MarkerArray()
        for idx, name in enumerate(self.names):
            msg.markers.append(self.make_marker(name, idx, now))
        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        FormationTrajectoryViz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
