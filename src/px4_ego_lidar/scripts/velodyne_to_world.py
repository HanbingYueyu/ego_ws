#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Transform Velodyne PointCloud2 from sensor frame to fixed world (same as odom_to_tf parent)."""
import rospy
import tf2_ros
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def main():
    rospy.init_node("velodyne_to_world")
    target_frame = rospy.get_param("~target_frame", "world")
    in_topic = rospy.get_param("~cloud_in", "cloud_in")
    out_topic = rospy.get_param("~cloud_out", "cloud_out")

    tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher(out_topic, PointCloud2, queue_size=2)

    def _transform_point(x, y, z, t):
        tx = t.translation.x
        ty = t.translation.y
        tz = t.translation.z
        qx = t.rotation.x
        qy = t.rotation.y
        qz = t.rotation.z
        qw = t.rotation.w

        # Quaternion rotation: v' = q * v * q^-1
        # Optimized form (assumes q normalized)
        # t = 2 * cross(q_vec, v)
        # v' = v + qw * t + cross(q_vec, t)
        t2x = 2.0 * (qy * z - qz * y)
        t2y = 2.0 * (qz * x - qx * z)
        t2z = 2.0 * (qx * y - qy * x)

        rx = x + qw * t2x + (qy * t2z - qz * t2y)
        ry = y + qw * t2y + (qz * t2x - qx * t2z)
        rz = z + qw * t2z + (qx * t2y - qy * t2x)

        return (rx + tx, ry + ty, rz + tz)

    def _transform_cloud(msg, trans):
        # Keep fields; rewrite only x,y,z in-place
        points = pc2.read_points(msg, field_names=None, skip_nans=True)

        # Determine indices of x,y,z fields
        field_names = [f.name for f in msg.fields]
        try:
            ix = field_names.index("x")
            iy = field_names.index("y")
            iz = field_names.index("z")
        except ValueError:
            rospy.logwarn_throttle(2.0, "velodyne_to_world: cloud has no x/y/z fields")
            return None

        t = trans.transform
        out_points = []
        for p in points:
            p = list(p)
            x, y, z = float(p[ix]), float(p[iy]), float(p[iz])
            nx, ny, nz = _transform_point(x, y, z, t)
            if not (math.isfinite(nx) and math.isfinite(ny) and math.isfinite(nz)):
                continue
            p[ix], p[iy], p[iz] = nx, ny, nz
            out_points.append(tuple(p))

        out = pc2.create_cloud(msg.header, msg.fields, out_points)
        out.header.stamp = msg.header.stamp
        out.header.frame_id = target_frame
        return out

    def cb(msg):
        src = msg.header.frame_id
        if not src:
            rospy.logwarn_throttle(5.0, "velodyne_to_world: empty frame_id, skip")
            return
        try:
            if msg.header.stamp == rospy.Time():
                trans = tf_buffer.lookup_transform(
                    target_frame, src, rospy.Time(0), rospy.Duration(0.15)
                )
            else:
                trans = tf_buffer.lookup_transform(
                    target_frame, src, msg.header.stamp, rospy.Duration(0.15)
                )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            try:
                trans = tf_buffer.lookup_transform(target_frame, src, rospy.Time(0), rospy.Duration(0.15))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(2.0, "velodyne_to_world: no TF %s -> %s: %s", src, target_frame, e)
                return

        out = _transform_cloud(msg, trans)
        if out is None:
            return
        pub.publish(out)

    rospy.Subscriber(in_topic, PointCloud2, cb, queue_size=2)
    rospy.loginfo(
        "velodyne_to_world: %s -> %s (target_frame=%s)", in_topic, out_topic, target_frame
    )
    rospy.spin()


if __name__ == "__main__":
    main()
