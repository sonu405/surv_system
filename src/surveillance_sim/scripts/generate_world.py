#!/usr/bin/env python3
import os
import math

# Gazebo Transport topic for each camera: "<camera_name>/image".
# These must match the bridge arguments in sim_launch.py.
def camera_topic(name):
    return f"{name}/image"


def create_box_link(name, x, y, z, dx, dy, dz, yaw=0.0, pitch=0.0, color="0.55 0.55 0.55 1"):
    return f"""
      <link name="{name}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 {pitch:.4f} {yaw:.4f}</pose>
        <collision name="{name}_col">
          <geometry><box><size>{dx:.3f} {dy:.3f} {dz:.3f}</size></box></geometry>
        </collision>
        <visual name="{name}_vis">
          <geometry><box><size>{dx:.3f} {dy:.3f} {dz:.3f}</size></box></geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>"""


def create_wall_segment(idx, x1, y1, x2, y2, height=3.0, thickness=0.3, color="0.65 0.65 0.65 1"):
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    yaw = math.atan2(dy, dx)
    return create_box_link(f"wall_{idx}", cx, cy, height / 2.0, length, thickness, height, yaw=yaw, color=color)


def create_camera_model(name, x, y, z, target_x, target_y, target_z=1.0):
    dx = target_x - x
    dy = target_y - y
    dz = z - target_z
    horiz_dist = math.hypot(dx, dy)

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(dz, horiz_dist)

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 {pitch:.4f} {yaw:.4f}</pose>
      <link name="camera_link">
        <visual name="cam_housing">
          <geometry><box><size>0.35 0.15 0.15</size></box></geometry>
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.85 0.85 0.85 1</diffuse>
          </material>
        </visual>
        <visual name="cam_lens">
          <pose>0.18 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.05</radius><length>0.05</length></cylinder></geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <sensor type="camera" name="{name}_sensor">
          <topic>{camera_topic(name)}</topic>
          <update_rate>20.0</update_rate>
          <camera name="head">
            <horizontal_fov>1.22</horizontal_fov>
            <image>
              <width>640</width>
              <height>640</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.2</near>
              <far>50.0</far>
            </clip>
          </camera>
        </sensor>
      </link>
    </model>"""


# Modern Gazebo loads nothing implicitly: physics, sensors (rendering),
# the GUI scene feed and user commands are all separate system plugins.
WORLD_SYSTEMS = """
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
"""

# Inline sun + ground so the world loads offline (model:// would go through Fuel).
WORLD_ENVIRONMENT = """
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

ACTOR_MESH = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"


def generate_world():
    links = []

    perimeter_pts = [
        (-16, 10), (-10, 14), (6, 13), (12, 10), (18, 6),
        (19, -4), (14, -11), (6, -13), (-4, -12), (-14, -9), (-18, -2)
    ]
    for i in range(len(perimeter_pts)):
        x1, y1 = perimeter_pts[i]
        x2, y2 = perimeter_pts[(i + 1) % len(perimeter_pts)]
        links.append(create_wall_segment(f"perim_{i}", x1, y1, x2, y2, height=2.5))

    links.append(create_wall_segment("baffle_top", -8, 13.5, -8, 9.5, height=2.5))
    links.append(create_wall_segment("baffle_bot1", -3, -12, -3, -8.5, height=2.5))
    links.append(create_wall_segment("baffle_bot2", 4, -12.5, 4, -9.0, height=2.5))

    links.append(create_box_link("shop_body", -5.0, 2.0, 1.75, 5.0, 4.0, 3.5, color="0.5 0.5 0.5 1"))
    links.append(create_box_link("shop_canopy", -2.0, 2.0, 2.8, 1.5, 4.2, 0.25, color="0.6 0.6 0.6 1"))

    pillars = [
        ("pillar_1", -1.0, -2.0, 5.5),
        ("pillar_2",  3.5, -1.0, 6.0),
        ("pillar_3",  5.5,  3.5, 7.5),
        ("pillar_4",  4.5, -4.5, 5.5),
    ]
    for name, px, py, h in pillars:
        links.append(create_box_link(name, px, py, h / 2.0, 1.2, 1.2, h, color="0.48 0.48 0.48 1"))

    links.append(create_box_link("shed_sw", -13.5, -4.5, 1.75, 3.5, 2.5, 3.5, yaw=0.4, color="0.42 0.42 0.42 1"))
    links.append(create_box_link("shed_se",  13.0, -5.5, 2.25, 5.0, 4.0, 4.5, yaw=-0.5, color="0.45 0.38 0.32 1"))
    links.append(create_box_link("shed_ne",  11.5,  6.5, 1.5,  3.0, 2.5, 3.0, yaw=-0.3, color="0.52 0.52 0.52 1"))

    cameras_xml = [
        create_camera_model("cam_pillar_north", 4.7, 3.2, 7.2, target_x=0.5, target_y=-2.5, target_z=1.0),
        create_camera_model("cam_shed_sw", -11.8, -3.8, 3.8, target_x=2.0, target_y=-1.0, target_z=1.0),
        create_camera_model("cam_building_se", 10.5, -3.8, 4.8, target_x=1.0, target_y=-1.0, target_z=1.0)
    ]

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="surveillance_arena">
{WORLD_SYSTEMS}
{WORLD_ENVIRONMENT}
    <model name="arena_structures">
      <static>true</static>
      {"".join(links)}
    </model>

    {"".join(cameras_xml)}

    <actor name="walking_person">
      <skin>
        <filename>{ACTOR_MESH}</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>{ACTOR_MESH}</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint><time>0.0</time><pose>-2.5 -4.0 0 0 0 0.5</pose></waypoint>
          <waypoint><time>4.0</time><pose>1.5 -1.5 0 0 0 0.8</pose></waypoint>
          <waypoint><time>8.0</time><pose>2.5 2.0 0 0 0 2.5</pose></waypoint>
          <waypoint><time>12.0</time><pose>-1.0 0.5 0 0 0 -1.8</pose></waypoint>
          <waypoint><time>16.0</time><pose>-2.5 -4.0 0 0 0 -1.0</pose></waypoint>
        </trajectory>
      </script>
    </actor>

  </world>
</sdf>
"""
    out_dir = os.path.join(os.path.dirname(__file__), "..", "worlds")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "surveillance_arena.world")

    with open(out_path, "w") as f:
        f.write(sdf_content)
    print(f"Generated world at: {os.path.abspath(out_path)}")


if __name__ == "__main__":
    generate_world()