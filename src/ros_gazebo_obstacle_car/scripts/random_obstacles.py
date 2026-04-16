#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def box_sdf(name, sx, sy, sz, color):
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{color}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def cylinder_sdf(name, radius, length, color):
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <cylinder><radius>{radius}</radius><length>{length}</length></cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder><radius>{radius}</radius><length>{length}</length></cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{color}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def safe_spawn(spawn_srv, name, sdf, pose):
    try:
        spawn_srv(name, sdf, '', pose, 'world')
        return True
    except Exception as exc:
        rospy.logwarn('Failed to spawn %s: %s', name, exc)
        return False


def make_marker(marker_id, marker_type, x, y, z, sx, sy, sz, r, g, b, a=1.0, ns='simulation_world'):
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz
    marker.color = ColorRGBA(r, g, b, a)
    return marker


def main():
    rospy.init_node('football_random_obstacles')

    seed = rospy.get_param('~seed', 42)
    obstacle_count = int(rospy.get_param('~obstacle_count', 22))
    field_length = float(rospy.get_param('~field_length', 105.0))
    field_width = float(rospy.get_param('~field_width', 68.0))
    clear_radius = float(rospy.get_param('~clear_radius', 6.0))

    random.seed(seed)

    marker_pub = rospy.Publisher('/rviz_world_markers', MarkerArray, queue_size=1, latch=True)
    marker_array = MarkerArray()

    rospy.loginfo('Waiting for Gazebo services...')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    marker_array.markers.append(
        make_marker(0, Marker.CUBE, 0.0, 0.0, -0.01, field_length, field_width, 0.02, 0.15, 0.35, 0.15, 0.35, 'field')
    )

    spawned = 0
    max_trials = obstacle_count * 12

    for idx in range(obstacle_count):
        placed = False
        for _ in range(max_trials):
            x = random.uniform(-field_length * 0.45, field_length * 0.45)
            y = random.uniform(-field_width * 0.45, field_width * 0.45)

            if x * x + y * y < clear_radius * clear_radius:
                continue

            model_name = f'football_obstacle_{idx}'
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            is_box = random.random() < 0.6

            if is_box:
                sx = random.uniform(0.8, 1.8)
                sy = random.uniform(0.8, 1.8)
                sz = random.uniform(1.2, 3.2)
                pose.position.z = sz * 0.5
                sdf = box_sdf(model_name, sx, sy, sz, 'Gazebo/Orange')
            else:
                radius = random.uniform(0.35, 0.8)
                length = random.uniform(1.0, 2.8)
                pose.position.z = length * 0.5
                sdf = cylinder_sdf(model_name, radius, length, 'Gazebo/Blue')

            if safe_spawn(spawn_srv, model_name, sdf, pose):
                spawned += 1
                placed = True
                if is_box:
                    marker_array.markers.append(
                        make_marker(idx + 1, Marker.CUBE, x, y, sz * 0.5, sx, sy, sz, 0.85, 0.45, 0.2, 0.9)
                    )
                else:
                    marker_array.markers.append(
                        make_marker(
                            idx + 1,
                            Marker.CYLINDER,
                            x,
                            y,
                            length * 0.5,
                            radius * 2.0,
                            radius * 2.0,
                            length,
                            0.2,
                            0.45,
                            0.9,
                            0.9,
                        )
                    )
                break

        if not placed:
            rospy.logwarn('Could not place obstacle %d after many trials.', idx)

    rospy.loginfo('Football obstacles spawned: %d/%d (seed=%d)', spawned, obstacle_count, seed)
    marker_pub.publish(marker_array)

    # Keep the node alive and refresh markers so RViz subscribers that connect late still see obstacles.
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
      marker_pub.publish(marker_array)
      rate.sleep()


if __name__ == '__main__':
    main()
