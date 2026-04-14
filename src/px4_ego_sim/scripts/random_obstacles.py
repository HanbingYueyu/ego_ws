#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_obstacles():
    rospy.init_node('spawn_random_obstacles')
    rospy.loginfo("Waiting for Gazebo spawn service...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
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
        
        # 调用服务生成模型
        try:
            spawn_model_prox(name, sdf, "", pose, "world")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn failed: {e}")
            
        rospy.sleep(0.05) # 稍微延时，防止 Gazebo 崩溃
        
    rospy.loginfo("All obstacles spawned successfully!")

if __name__ == '__main__':
    try:
        spawn_obstacles()
    except rospy.ROSInterruptException:
        pass
