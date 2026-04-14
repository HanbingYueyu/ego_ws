#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
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

if __name__ == '__main__':
    rospy.init_node('world_builder')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    marker_pub = rospy.Publisher('/rviz_world_markers', MarkerArray, queue_size=1, latch=True)
    marker_array = MarkerArray()

    # 1. 生成足球场 (Gazebo/Green)
    field_sdf = create_box_sdf("football_field", 105.0, 68.0, 0.02, "Gazebo/Green")
    try: spawn_model("football_field", field_sdf, "", Pose(position=Point(x=0, y=0, z=-0.01)), "world")
    except: pass

    rviz_field = create_rviz_marker(0, 1, 0, 0, -0.05, 105.0, 68.0, 0.1, 0.1, 0.6, 0.1, 0.8)
    marker_array.markers.append(rviz_field)

    # 2. 生成障碍物 (Gazebo/Red)
    for i in range(25):
        x = random.uniform(-40, 40)
        y = random.uniform(-30, 30)
        if -3 < x < 3 and -3 < y < 3: continue 

        obs_sdf = create_box_sdf(f"obstacle_{i}", 1.0, 1.0, 5.0, "Gazebo/Red")
        try: spawn_model(f"obstacle_{i}", obs_sdf, "", Pose(position=Point(x=x, y=y, z=0)), "world")
        except: pass

        rviz_obs = create_rviz_marker(i+1, 1, x, y, 2.5, 1.0, 1.0, 5.0, 0.8, 0.2, 0.2, 0.9)
        marker_array.markers.append(rviz_obs)

    rospy.sleep(1.0)
    marker_pub.publish(marker_array)
    rospy.loginfo("足球场及障碍物生成完毕！")
    rospy.spin()
