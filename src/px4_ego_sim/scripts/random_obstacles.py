#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pcl2


def sample_box_surface_points(x, y, z, sx, sy, sz, step=0.2):
  points = []
  x_min = x - sx / 2.0
  x_max = x + sx / 2.0
  y_min = y - sy / 2.0
  y_max = y + sy / 2.0
  z_min = 0.0
  z_max = sz

  def grid_values(start, end):
    value = start
    while value <= end + 1e-6:
      yield value
      value += step

  for xi in [x_min, x_max]:
    for yi in grid_values(y_min, y_max):
      for zi in grid_values(z_min, z_max):
        points.append([xi, yi, zi])

  for yi in [y_min, y_max]:
    for xi in grid_values(x_min, x_max):
      for zi in grid_values(z_min, z_max):
        points.append([xi, yi, zi])

  for zi in [z_min, z_max]:
    for xi in grid_values(x_min, x_max):
      for yi in grid_values(y_min, y_max):
        points.append([xi, yi, zi])

  return points

def spawn_obstacles():
    rospy.init_node('spawn_random_obstacles')
    rospy.loginfo("Waiting for Gazebo spawn service...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    cloud_pub = rospy.Publisher('/map_generator/global_cloud', PointCloud2, queue_size=1, latch=True)
    rospy.loginfo("Connected to Gazebo! Spawning obstacles...")

    # 定义一个基础的长方体 SDF 模板 (静态，不会被撞飞)
    sdf_template = """<?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="{name}">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          </collision>
          <visual name="visual">
            <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/WoodPallet</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>"""

    num_obstacles = 25 # 生成 25 个障碍物
    cloud_points = []
    
    for i in range(num_obstacles):
        # 在无人机正前方 2m~30m，左右各 10m 的“足球场”范围内随机分布
        x = random.uniform(2.0, 30.0) 
        y = random.uniform(-10.0, 10.0)
        
        # 随机大小 (宽和深 0.5~1.5m，高度 2.0~6.0m)
        sx = random.uniform(0.5, 1.5)
        sy = random.uniform(0.5, 1.5)
        sz = random.uniform(2.0, 6.0)
        
        # 物体中心的高度应为总高度的一半，这样才能贴合地面
        z = sz / 2.0 

        name = f"obstacle_{i}"
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        sdf = sdf_template.format(name=name, sx=sx, sy=sy, sz=sz)
        cloud_points.extend(sample_box_surface_points(x, y, z, sx, sy, sz))
        
        # 调用服务生成模型
        try:
            spawn_model_prox(name, sdf, "", pose, "world")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn failed: {e}")
            
        rospy.sleep(0.05) # 稍微延时，防止 Gazebo 崩溃
        
    rospy.loginfo("All obstacles spawned successfully!")

    header = Header()
    header.frame_id = "world"
    cloud_msg = pcl2.create_cloud_xyz32(header, cloud_points)

    for _ in range(10):
      header.stamp = rospy.Time.now()
      cloud_msg.header = header
      cloud_pub.publish(cloud_msg)
      rospy.sleep(0.3)

    rospy.loginfo("Published matching global obstacle cloud with %d points.", len(cloud_points))

if __name__ == '__main__':
    try:
        spawn_obstacles()
    except rospy.ROSInterruptException:
        pass
