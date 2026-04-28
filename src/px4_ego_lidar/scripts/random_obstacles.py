#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

def create_box_sdf(name, size_x, size_y, size_z, gazebo_material):
    """使用 Gazebo 官方材质库强制渲染颜色"""
    return f"""<?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="{name}">
        <static>true</static>
        <link name="link">
          <pose>0 0 {size_z/2.0} 0 0 0</pose>
          <visual name="visual">
            <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
            <material>
              <script>
                <name>{gazebo_material}</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <collision name="collision">
            <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
          </collision>
        </link>
      </model>
    </sdf>"""

def create_rviz_marker(marker_id, m_type, x, y, z, sx, sy, sz, r, g, b, a=1.0):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "simulation_world"
    marker.id = marker_id
    marker.type = m_type
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.x = sx; marker.scale.y = sy; marker.scale.z = sz
    marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = a
    return marker

def inside_box(x, y, x_min, x_max, y_min, y_max):
    return x_min <= x <= x_max and y_min <= y <= y_max

def box_points(cx, cy, size_x, size_y, size_z, resolution):
    points = []
    x0 = cx - size_x / 2.0
    x1 = cx + size_x / 2.0
    y0 = cy - size_y / 2.0
    y1 = cy + size_y / 2.0
    z0 = 0.0
    z1 = size_z

    nx = max(1, int(size_x / resolution))
    ny = max(1, int(size_y / resolution))
    nz = max(1, int(size_z / resolution))

    for ix in range(nx + 1):
        x = x0 + size_x * ix / float(nx)
        for iy in range(ny + 1):
            y = y0 + size_y * iy / float(ny)
            points.append((x, y, z0))
            points.append((x, y, z1))

    for ix in range(nx + 1):
        x = x0 + size_x * ix / float(nx)
        for iz in range(nz + 1):
            z = z0 + size_z * iz / float(nz)
            points.append((x, y0, z))
            points.append((x, y1, z))

    for iy in range(ny + 1):
        y = y0 + size_y * iy / float(ny)
        for iz in range(nz + 1):
            z = z0 + size_z * iz / float(nz)
            points.append((x0, y, z))
            points.append((x1, y, z))

    return points

if __name__ == '__main__':
    rospy.init_node('world_builder')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    marker_pub = rospy.Publisher('/rviz_world_markers', MarkerArray, queue_size=1, latch=True)
    global_cloud_pub = rospy.Publisher('/map_generator/global_cloud', PointCloud2, queue_size=1, latch=True)
    marker_array = MarkerArray()
    obstacle_count = int(rospy.get_param("~obstacle_count", 25))
    obstacle_size_x = float(rospy.get_param("~obstacle_size_x", 1.0))
    obstacle_size_y = float(rospy.get_param("~obstacle_size_y", 1.0))
    obstacle_size_z = float(rospy.get_param("~obstacle_size_z", 5.0))
    global_cloud_resolution = float(rospy.get_param("~global_cloud_resolution", 0.2))
    x_min = float(rospy.get_param("~x_min", -40.0))
    x_max = float(rospy.get_param("~x_max", 40.0))
    y_min = float(rospy.get_param("~y_min", -30.0))
    y_max = float(rospy.get_param("~y_max", 30.0))
    protected_x_min = float(rospy.get_param("~protected_x_min", -12.5))
    protected_x_max = float(rospy.get_param("~protected_x_max", 6.5))
    protected_y_min = float(rospy.get_param("~protected_y_min", -3.5))
    protected_y_max = float(rospy.get_param("~protected_y_max", 3.5))
    max_attempts = int(rospy.get_param("~max_attempts", 1000))

    # 1. 生成足球场 (Gazebo/Green)
    field_sdf = create_box_sdf("football_field", 105.0, 68.0, 0.02, "Gazebo/Green")
    try: spawn_model("football_field", field_sdf, "", Pose(position=Point(x=0, y=0, z=-0.01)), "world")
    except: pass

    rviz_field = create_rviz_marker(0, 1, 0, 0, -0.05, 105.0, 68.0, 0.1, 0.1, 0.6, 0.1, 0.8)
    marker_array.markers.append(rviz_field)

    # 2. 生成障碍物 (Gazebo/Red)
    spawned = 0
    attempts = 0
    while spawned < obstacle_count and attempts < max_attempts:
        attempts += 1
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        if inside_box(x, y, protected_x_min, protected_x_max, protected_y_min, protected_y_max):
            continue

        obs_sdf = create_box_sdf(
            f"obstacle_{spawned}", obstacle_size_x, obstacle_size_y, obstacle_size_z, "Gazebo/Red"
        )
        try: spawn_model(f"obstacle_{spawned}", obs_sdf, "", Pose(position=Point(x=x, y=y, z=0)), "world")
        except: pass

        rviz_obs = create_rviz_marker(
            spawned + 1, 1, x, y, obstacle_size_z / 2.0,
            obstacle_size_x, obstacle_size_y, obstacle_size_z, 0.8, 0.2, 0.2, 0.9
        )
        marker_array.markers.append(rviz_obs)
        spawned += 1

    cloud_points = []
    for marker in marker_array.markers:
        if marker.id == 0:
            continue
        cloud_points.extend(
            box_points(
                marker.pose.position.x,
                marker.pose.position.y,
                obstacle_size_x,
                obstacle_size_y,
                obstacle_size_z,
                global_cloud_resolution,
            )
        )

    cloud_header = Header()
    cloud_header.stamp = rospy.Time.now()
    cloud_header.frame_id = "world"
    global_cloud = pc2.create_cloud_xyz32(cloud_header, cloud_points)

    rospy.sleep(1.0)
    marker_pub.publish(marker_array)
    global_cloud_pub.publish(global_cloud)
    rospy.loginfo("足球场及障碍物生成完毕！障碍物数量: %d, 保护区: x[%.1f, %.1f], y[%.1f, %.1f]",
                  spawned, protected_x_min, protected_x_max, protected_y_min, protected_y_max)
    rospy.spin()
