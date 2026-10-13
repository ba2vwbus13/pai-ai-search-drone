#!/usr/bin/env python3
"""高度別の検出性能テスト用ワールドを生成する。

高度 10/15/20/25/30 m に真下向きカメラを固定し、各カメラの真下に
車2台・立っている人2人・倒れている人1人を配置する。
PX4 は使わず gz sim 単体で起動し、各カメラの画像を YOLO にかける。
"""
from pathlib import Path

FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
ALTITUDES = [10, 15, 20, 25, 30]
GROUP_SPACING = 80.0

# (モデル名, 東 x, 北 y, ヨー) — グループ中心からのオフセット [m]
TARGETS = [
    ('SUV', -6.0, 4.0, 0.3),
    ('Hatchback blue', 6.0, -5.0, 1.8),
    ('Casual female', -3.0, -6.0, 0.0),
    ('Walking person', 4.0, 5.0, 2.0),
    ('Rescue Randy', 0.0, 0.0, 0.7),
]

HEADER = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="alt_test">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.6 0.7 0.9 1</background><shadows>true</shadows></scene>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose><cast_shadows>true</cast_shadows>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>0.3 0.4 -0.85</direction>
    </light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>
        <material><diffuse>0.45 0.65 0.3 1</diffuse><specular>0 0 0 1</specular></material></visual>
    </link></model>
"""


def main():
    parts = [HEADER]
    for i, alt in enumerate(ALTITUDES):
        cx = i * GROUP_SPACING
        parts.append(f"""    <include><uri>model://mono_cam</uri><name>cam_{alt}m</name>
      <pose>{cx} 0 {alt} 0 1.5707 0</pose><static>true</static></include>
""")
        for j, (name, dx, dy, yaw) in enumerate(TARGETS):
            parts.append(f"""    <include><uri>{FUEL}{name.replace(' ', '%20')}</uri><name>t{alt}_{j}</name>
      <pose>{cx + dx} {dy} 0 0 0 {yaw}</pose><static>true</static></include>
""")
    parts.append('  </world>\n</sdf>\n')
    out = Path(__file__).with_name('altitude_test.sdf')
    out.write_text(''.join(parts))
    print(out)


if __name__ == '__main__':
    main()
