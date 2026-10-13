#!/usr/bin/env python3
"""捜索シナリオのワールド okinawa_sar.sdf と正解データ targets.json を生成する。

想定: 災害発生後の沖縄県内の学校周辺。離陸地点(原点)の北側 120m 四方を捜索範囲とし、
倒れている要救助者・屋外に取り残された人・放置車両・家屋・樹木を配置する。

座標: Gazebo のワールド座標(ENU: x=東, y=北)[m]。
PX4 のローカル座標(NED)では north = y, east = x になる(機体は原点から離陸)。

実行: python3 make_world.py   → 同じフォルダに okinawa_sar.sdf と targets.json
"""
import json
from pathlib import Path

FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
WORLD = 'okinawa_sar'
HOME_LAT, HOME_LON = 26.5246226, 128.0297311   # 離陸地点(沖縄県内)

# 捜索対象(人)。(id, モデル, 東 x, 北 y, ヨー, 説明)
PERSONS = [
    ('P1', 'Rescue Randy', -35.0, 42.0, 0.5, '倒れている人'),
    ('P2', 'Rescue Randy', 28.0, 97.0, 2.0, '倒れている人'),
    ('P3', 'Casual female', 12.0, 58.0, 1.0, '立っている人'),
    ('P4', 'Male visitor', -22.0, 112.0, 0.0, '立っている人'),
    ('P5', 'Walking person', 45.0, 27.0, 2.5, '歩いている人'),
    ('P6', 'Standing person', -48.0, 82.0, -1.0, '立っている人'),
]

# 背景物(検出対象ではないが、誤検出・遮蔽の要因になる)
VEHICLES = [
    ('SUV', 2.5, 35.0, 0.0),
    ('Pickup', -2.5, 88.0, 3.14),
    ('Hatchback blue', 40.0, 118.0, 0.8),
    ('Prius Hybrid', -45.0, 125.0, 0.0),
    ('Ambulance', 9.0, -8.0, 1.57),
]
# 建物(東 x, 北 y, 幅, 奥行き, 高さ, ヨー, 屋根の色)。Fuel の家モデルは真上から黒く写るため箱で表現
BUILDINGS = [(-30.0, 63.0, 12.0, 9.0, 6.0, 0.0, '0.62 0.30 0.25'),
             (35.0, 72.0, 10.0, 14.0, 7.0, 0.0, '0.55 0.55 0.60'),
             (-28.0, 92.0, 8.0, 8.0, 5.0, 0.3, '0.75 0.72 0.62'),
             (48.0, 100.0, 9.0, 7.0, 5.0, 0.0, '0.35 0.45 0.60')]
TREES = [(-15.0, 20.0), (50.0, 50.0), (15.0, 128.0), (-55.0, 100.0), (20.0, 85.0)]

HEADER = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- 自動生成: worlds/make_world.py。直接編集しないこと -->
<sdf version="1.9">
  <world name="{WORLD}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <sky/>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1.0</ambient>
      <background>0.6 0.7 0.9 1.0</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.3 0.4 -0.85</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{HOME_LAT}</latitude_deg>
      <longitude_deg>{HOME_LON}</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <!-- 地面(不透明。PX4 標準の lawn は半透明で真下カメラに写らない) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
          <material><diffuse>0.45 0.62 0.32 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
      </link>
    </model>
"""


def flat_box(name, x, y, sx, sy, rgb, z=0.01):
    """地面に貼る薄い板(道路・ヘリパッドなど)。当たり判定なし。"""
    return f"""    <model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>
      <link name="link"><visual name="v"><cast_shadows>false</cast_shadows>
        <geometry><box><size>{sx} {sy} 0.02</size></box></geometry>
        <material><diffuse>{rgb} 1</diffuse><specular>0 0 0 1</specular></material>
      </visual></link></model>
"""


def building(name, x, y, sx, sy, h, yaw, rgb):
    """箱型の建物(壁 + 屋根)。"""
    return f"""    <model name="{name}"><static>true</static><pose>{x} {y} {h / 2} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name="wall"><geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><diffuse>0.85 0.83 0.78 1</diffuse><specular>0.1 0.1 0.1 1</specular></material></visual>
        <visual name="roof"><pose>0 0 {h / 2 + 0.15} 0 0 0</pose>
          <geometry><box><size>{sx + 0.6} {sy + 0.6} 0.3</size></box></geometry>
          <material><diffuse>{rgb} 1</diffuse><specular>0.1 0.1 0.1 1</specular></material></visual>
      </link></model>
"""


def include(uri_name, inst, x, y, yaw):
    return f"""    <include><uri>{FUEL}{uri_name.replace(' ', '%20')}</uri><name>{inst}</name>
      <pose>{x} {y} 0 0 0 {yaw}</pose><static>true</static></include>
"""


def main():
    here = Path(__file__).resolve().parent
    parts = [HEADER]
    # 道路(南北の幹線と東西の生活道路)とヘリパッド
    parts.append(flat_box('road_ns', 0, 70, 9, 170, '0.35 0.35 0.37'))
    parts.append(flat_box('road_ew', 0, 75, 130, 6, '0.40 0.40 0.42'))
    parts.append(flat_box('helipad', 0, 0, 6, 6, '0.25 0.25 0.25', z=0.012))
    parts.append(flat_box('helipad_h1', -1.2, 0, 0.5, 3, '0.95 0.95 0.95', z=0.024))
    parts.append(flat_box('helipad_h2', 1.2, 0, 0.5, 3, '0.95 0.95 0.95', z=0.024))
    parts.append(flat_box('helipad_h3', 0, 0, 2.4, 0.5, '0.95 0.95 0.95', z=0.024))

    for pid, model, x, y, yaw, _ in PERSONS:
        parts.append(include(model, f'person_{pid}', x, y, yaw))
    for i, (model, x, y, yaw) in enumerate(VEHICLES):
        parts.append(include(model, f'vehicle_{i}', x, y, yaw))
    for i, b in enumerate(BUILDINGS):
        parts.append(building(f'building_{i}', *b))
    for i, (x, y) in enumerate(TREES):
        parts.append(include('Oak tree', f'tree_{i}', x, y, 0.0))
    parts.append('  </world>\n</sdf>\n')

    (here / f'{WORLD}.sdf').write_text(''.join(parts))
    targets = {
        'world': WORLD,
        'home_lat': HOME_LAT,
        'home_lon': HOME_LON,
        'frame': 'NED(north=y, east=x) [m] — PX4 ローカル座標と同じ',
        'persons': [dict(id=pid, model=m, north=y, east=x, note=note)
                    for pid, m, x, y, _, note in PERSONS],
        'vehicles': [dict(model=m, north=y, east=x) for m, x, y, _ in VEHICLES],
    }
    (here / 'targets.json').write_text(json.dumps(targets, ensure_ascii=False, indent=2))
    print('wrote', here / f'{WORLD}.sdf', here / 'targets.json')


if __name__ == '__main__':
    main()
