#!/usr/bin/env python3
"""Generate the Gazebo world SDF from the shared synthetic-world constants.

The wall, pillars, ground, defect decals, and robot spawn pose are all
derived from defect_detection.simulation.world_constants, so the Gazebo
scene always matches the LAS scans and the ground-truth detections.

Usage:
    python3 scripts/generate_gazebo_world.py
"""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'defect_detection'))

from defect_detection.simulation.world_constants import (  # noqa: E402
    DEFECTS,
    PILLAR_LATERALS,
    PILLAR_OFFSET_FROM_WALL,
    PILLAR_RADIUS,
    WALL_HEIGHT,
    WALL_LATERAL_MAX,
    WALL_LATERAL_MIN,
    WALL_PLANE_X,
    WALL_THICKNESS,
)

OUTPUT = REPO_ROOT / 'src' / 'defect_detection' / 'worlds' / 'inspection_site.sdf'

CONCRETE = '0.62 0.63 0.60 1'
CONCRETE_DARK = '0.42 0.44 0.45 1'
GROUND_COLOR = '0.38 0.41 0.42 1'
CRACK_COLOR = '0.12 0.12 0.13 1'
SPALL_COLOR = '0.28 0.29 0.31 1'
REBAR_COLOR = '0.45 0.22 0.10 1'


def material(color):
    return f"""
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>"""


def box_visual(name, pose, size, color):
    return f"""
        <visual name="{name}">
          <pose>{pose}</pose>
          <geometry>
            <box><size>{size}</size></box>
          </geometry>{material(color)}
        </visual>"""


def defect_visuals():
    face_x = WALL_PLANE_X - 0.01
    visuals = []
    for class_id, lateral, height, size, _ in DEFECTS:
        if class_id == 'crack':
            visuals.append(box_visual(
                f'defect_{class_id}',
                f'{face_x} {lateral} {height} 0.785 0 0',
                f'0.02 {size * 0.95:.3f} 0.06',
                CRACK_COLOR,
            ))
        elif class_id == 'spalling':
            visuals.append(box_visual(
                f'defect_{class_id}',
                f'{face_x} {lateral} {height} 0 0 0',
                f'0.02 {size:.3f} {size * 0.6:.3f}',
                SPALL_COLOR,
            ))
            visuals.append(box_visual(
                f'defect_{class_id}_core',
                f'{face_x - 0.005} {lateral - size / 4:.3f} {height} 0 0 0',
                f'0.02 {size / 2:.3f} {size / 3:.3f}',
                CRACK_COLOR,
            ))
        else:  # exposed_rebar
            for index, offset in enumerate((-size / 3, 0.0, size / 3)):
                visuals.append(box_visual(
                    f'defect_{class_id}_{index}',
                    f'{face_x} {lateral + offset:.3f} {height} 0 0 0',
                    f'0.02 0.05 {size:.3f}',
                    REBAR_COLOR,
                ))
    return ''.join(visuals)


def pillar_visuals():
    pillar_x = WALL_PLANE_X - PILLAR_OFFSET_FROM_WALL
    visuals = []
    for index, lateral in enumerate(PILLAR_LATERALS):
        visuals.append(f"""
        <visual name="pillar_{index}">
          <pose>{pillar_x} {lateral} {WALL_HEIGHT / 2} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{PILLAR_RADIUS}</radius>
              <length>{WALL_HEIGHT}</length>
            </cylinder>
          </geometry>{material(CONCRETE_DARK)}
        </visual>
        <collision name="pillar_{index}_collision">
          <pose>{pillar_x} {lateral} {WALL_HEIGHT / 2} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{PILLAR_RADIUS}</radius>
              <length>{WALL_HEIGHT}</length>
            </cylinder>
          </geometry>
        </collision>""")
    return ''.join(visuals)


def joint_visuals():
    """Horizontal formwork joints for visual texture on the wall face."""
    visuals = []
    wall_span = WALL_LATERAL_MAX - WALL_LATERAL_MIN
    for index in range(1, 4):
        z = WALL_HEIGHT * index / 4
        visuals.append(box_visual(
            f'wall_joint_{index}',
            f'{WALL_PLANE_X - 0.005} 0 {z} 0 0 0',
            f'0.01 {wall_span} 0.03',
            CONCRETE_DARK,
        ))
    return ''.join(visuals)


def world_sdf():
    wall_center_x = WALL_PLANE_X + WALL_THICKNESS / 2
    wall_span = WALL_LATERAL_MAX - WALL_LATERAL_MIN
    return f"""<?xml version="1.0"?>
<!--
  GENERATED FILE - do not edit by hand.
  Regenerate with: python3 scripts/generate_gazebo_world.py
  Geometry source: defect_detection/simulation/world_constants.py
-->
<sdf version="1.9">
  <world name="inspection_site">
    <physics name="default_physics" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.65 0.75 0.85 1</background>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>60 60</size></plane>
          </geometry>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>60 60</size></plane>
          </geometry>{material(GROUND_COLOR)}
        </visual>
      </link>
    </model>

    <model name="inspection_wall">
      <static>true</static>
      <link name="wall_link">
        <collision name="wall_collision">
          <pose>{wall_center_x} 0 {WALL_HEIGHT / 2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{WALL_THICKNESS} {wall_span} {WALL_HEIGHT}</size>
            </box>
          </geometry>
        </collision>{box_visual(
            'wall_visual',
            f'{wall_center_x} 0 {WALL_HEIGHT / 2} 0 0 0',
            f'{WALL_THICKNESS} {wall_span} {WALL_HEIGHT}',
            CONCRETE,
        )}{joint_visuals()}{pillar_visuals()}{defect_visuals()}
      </link>
    </model>

    <model name="spot_sim">
      <pose>0 0 0 0 0 0</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial>
          <mass>30.0</mass>
          <inertia>
            <ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>{box_visual(
            'body', '0 0 0.45 0 0 0', '0.8 0.35 0.22', '0.85 0.75 0.15 1',
        )}{box_visual(
            'leg_fl', '0.28 0.14 0.18 0 0 0', '0.06 0.06 0.36', '0.2 0.2 0.2 1',
        )}{box_visual(
            'leg_fr', '0.28 -0.14 0.18 0 0 0', '0.06 0.06 0.36', '0.2 0.2 0.2 1',
        )}{box_visual(
            'leg_rl', '-0.28 0.14 0.18 0 0 0', '0.06 0.06 0.36', '0.2 0.2 0.2 1',
        )}{box_visual(
            'leg_rr', '-0.28 -0.14 0.18 0 0 0', '0.06 0.06 0.36', '0.2 0.2 0.2 1',
        )}{box_visual(
            'camera_housing', '0.25 0 0.65 0 0 0', '0.10 0.16 0.06', '0.1 0.1 0.12 1',
        )}
        <sensor name="oak_rgbd" type="rgbd_camera">
          <pose>0.25 0 0.65 0 0 0</pose>
          <topic>oak/rgb</topic>
          <gz_frame_id>camera_optical_frame</gz_frame_id>
          <update_rate>10</update_rate>
          <camera>
            <horizontal_fov>1.25</horizontal_fov>
            <image>
              <width>640</width>
              <height>400</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>25.0</far>
            </clip>
          </camera>
          <always_on>true</always_on>
          <visualize>true</visualize>
        </sensor>
      </link>
      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <topic>/model/spot_sim/cmd_vel</topic>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system"
              name="gz::sim::systems::OdometryPublisher">
        <odom_frame>odom</odom_frame>
        <robot_base_frame>base_link</robot_base_frame>
        <odom_topic>/model/spot_sim/odometry</odom_topic>
        <tf_topic>/model/spot_sim/tf</tf_topic>
        <odom_publish_frequency>30</odom_publish_frequency>
      </plugin>
    </model>
  </world>
</sdf>
"""


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(world_sdf(), encoding='utf-8')
    print(OUTPUT)


if __name__ == '__main__':
    main()
