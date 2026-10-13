#!/usr/bin/env python3
"""カメラの傾き(真下 / 斜め)による人の検出性能の違いを調べるテストワールド。

各条件でカメラの注視点のまわりに 立っている人4人 + 倒れている人1人 を置く。
条件: (高度 [m], 真下からの傾き [deg])
"""
import math
from pathlib import Path

FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
CONDITIONS = [(8, 0), (8, 35), (12, 0), (12, 35), (18, 0), (18, 30), (18, 45)]
PEOPLE = [('Casual female', -2.5, -2.5, 0.3), ('Male visitor', 2.5, -2.5, 1.2),
          ('Walking person', -2.5, 2.5, 2.4), ('Standing person', 2.5, 2.5, -0.8),
          ('Rescue Randy', 0.0, 0.0, 0.7)]

HEADER = Path(__file__).with_name('make_altitude_test_world.py').read_text().split('HEADER = """')[1].split('"""')[0]


def main():
    parts = [HEADER.replace('alt_test', 'tilt_test')]
    for i, (alt, tilt) in enumerate(CONDITIONS):
        cx = i * 100.0
        look = alt * math.tan(math.radians(tilt))      # 注視点までの水平距離
        pitch = math.radians(90 - tilt)
        parts.append(f"""    <include><uri>model://mono_cam</uri><name>cam_{alt}m_{tilt}deg</name>
      <pose>{cx - look} 0 {alt} 0 {pitch} 0</pose><static>true</static></include>
""")
        for j, (name, dx, dy, yaw) in enumerate(PEOPLE):
            parts.append(f"""    <include><uri>{FUEL}{name.replace(' ', '%20')}</uri><name>p{i}_{j}</name>
      <pose>{cx + dx} {dy} 0 0 0 {yaw}</pose><static>true</static></include>
""")
    parts.append('  </world>\n</sdf>\n')
    out = Path(__file__).with_name('tilt_test.sdf')
    out.write_text(''.join(parts))
    print(out)


if __name__ == '__main__':
    main()
