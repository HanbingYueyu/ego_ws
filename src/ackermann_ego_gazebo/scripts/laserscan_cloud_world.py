#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LaserScan -> PointCloud2 in world frame (for ego_planner grid_map)."""
import rospy
import tf
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2


def main():
    rospy.init_node("laserscan_cloud_world")
    target_frame = rospy.get_param("~target_frame", "world")
    scan_in = rospy.get_param("~scan_in", "/scan")
    cloud_out = rospy.get_param("~cloud_out", "/scan_cloud_world")

    proj = LaserProjection()
    tf_listener = tf.TransformListener()
    pub = rospy.Publisher(cloud_out, PointCloud2, queue_size=2)

    def cb(scan: LaserScan):
        try:
            cloud_scan = proj.projectLaser(scan)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "projectLaser failed: %s", e)
            return
        src_frame = cloud_scan.header.frame_id
        if not src_frame:
            return

        try:
            tf_listener.waitForTransform(
                target_frame, src_frame, rospy.Time(0), rospy.Duration(0.2)
            )
            trans, rot = tf_listener.lookupTransform(target_frame, src_frame, rospy.Time(0))
            mat = tf.transformations.quaternion_matrix(rot)
            mat[0, 3] = trans[0]
            mat[1, 3] = trans[1]
            mat[2, 3] = trans[2]
        except Exception as e:
            rospy.logwarn_throttle(2.0, "scan->cloud tf %s->%s failed: %s", src_frame, target_frame, e)
            return

        pts_world = []
        for p in pc2.read_points(cloud_scan, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            v = mat.dot([x, y, z, 1.0])
            pts_world.append([v[0], v[1], v[2]])

        if not pts_world:
            return
        cloud_world = pc2.create_cloud_xyz32(cloud_scan.header, pts_world)
        cloud_world.header.frame_id = target_frame
        pub.publish(cloud_world)

    rospy.Subscriber(scan_in, LaserScan, cb, queue_size=2)
    rospy.loginfo("laserscan_cloud_world: %s -> %s (frame %s)", scan_in, cloud_out, target_frame)
    rospy.spin()


if __name__ == "__main__":
    main()
