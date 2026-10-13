#!/usr/bin/env python3
"""捜索ミッションノード: 巡回 → AI で発見 → 降下して確認 → 巡回に戻る。

状態遷移:
  INIT → ARMING → TAKEOFF → PATROL ─(人の候補を発見)→ APPROACH → DESCEND → OBSERVE
                              ↑                                              │
                              └──── RESUME ←──── CLIMB ←─(確定 / 誤検出として棄却)┘
  PATROL(全ルート完了)→ RETURN → LAND → DONE(報告書 JSON を保存)

2段階確認の考え方:
  巡回高度(既定 18m)では広く見渡せるが、人は小さくしか写らず誤検出もある。
  そこで候補を見つけたら、候補が画像の中央に来る位置(カメラが前方斜め下 30° を
  向いているので、候補の手前側)へ移動し、確認高度(既定 10m)まで降りて
  近距離の画像で再度 AI 判定する。一定以上の確信度で複数回検出できたら「確定」、
  できなければ「誤検出」として棄却する。確定位置は近距離での推定値の平均を使う。

入力: /ai_patrol/detections (detector_node), PX4 の位置・状態
出力: PX4 へのオフボード目標、/ai_patrol/mission_state (JSON, 表示用)、報告書 JSON
"""
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from .geo import footprint, look_offset, ned_to_latlon

STATE_JP = {
    'INIT': '準備中', 'ARMING': 'アーム', 'TAKEOFF': '離陸', 'PATROL': '巡回(AI 監視中)',
    'APPROACH': '候補へ移動', 'DESCEND': '降下して確認', 'OBSERVE': 'AI で再確認',
    'CLIMB': '上昇', 'RESUME': '巡回へ復帰', 'RETURN': '帰還', 'LAND': '着陸', 'DONE': '完了',
}


def coverage_route(center_n, center_e, side, swath, overlap):
    """芝刈り(往復)ルート。東西に飛ぶ線を南から北へ並べる。

    swath はカメラの横方向の撮影幅。隣の線と overlap の割合だけ重ねる。
    """
    half = side / 2.0
    spacing = swath * (1.0 - overlap)
    span = max(0.0, side - swath)
    n = max(1, math.ceil(span / spacing) + 1)
    norths = [center_n] if n == 1 else [
        center_n - span / 2 + i * span / (n - 1) for i in range(n)]
    route = []
    for i, nn in enumerate(norths):
        e0, e1 = (center_e - half, center_e + half) if i % 2 == 0 else (center_e + half, center_e - half)
        route += [(nn, e0), (nn, e1)]
    return route


class Target:
    def __init__(self, tid, north, east, conf, label):
        self.id = tid
        self.north = north            # 現在の推定位置
        self.east = east
        self.first_north = north      # 巡回高度で最初に推定した位置
        self.first_east = east
        self.first_conf = conf
        self.label = label
        self.status = 'pending'       # pending / inspecting / confirmed / rejected
        self.obs = []                 # 確認高度での (north, east, conf)
        self.found_time = time.time()

    def as_dict(self, lat0, lon0):
        lat, lon = ned_to_latlon(self.north, self.east, lat0, lon0)
        return dict(id=self.id, label=self.label, status=self.status,
                    north=round(self.north, 2), east=round(self.east, 2),
                    lat=round(lat, 7), lon=round(lon, 7),
                    first_north=round(self.first_north, 2), first_east=round(self.first_east, 2),
                    first_conf=round(self.first_conf, 3),
                    n_confirm=len(self.obs),
                    max_conf=round(max([o[2] for o in self.obs], default=0.0), 3))


class MissionNode(Node):
    def __init__(self):
        super().__init__('ai_mission')
        defaults = dict(
            area_center_north=70.0, area_center_east=0.0, area_size=120.0,
            patrol_alt=18.0, inspect_alt=10.0, hfov=1.74, cam_tilt_deg=30.0, overlap=0.2,
            patrol_speed=6.0, transit_speed=8.0, vertical_speed=2.0,
            candidate_conf=0.30, confirm_conf=0.50, confirm_count=2,
            merge_radius=7.0, observe_s=4.0, observe_timeout_s=8.0,
            accept_radius=1.5, home_lat=26.5246226, home_lon=128.0297311,
            report_dir=os.path.expanduser('~/work/pai_final/results'),
            target_labels=['person'],
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        c = self.cfg

        swath, _ = footprint(c['patrol_alt'], c['hfov'])
        self.route = coverage_route(c['area_center_north'], c['area_center_east'],
                                    c['area_size'], swath, c['overlap'])
        self.wp = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_state = self.create_publisher(String, '/ai_patrol/mission_state', 10)
        for t in ('/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_local_position'):
            self.create_subscription(VehicleLocalPosition, t, self._on_pos, qos)
        for t in ('/fmu/out/vehicle_status_v4', '/fmu/out/vehicle_status'):
            self.create_subscription(VehicleStatus, t, self._on_status, qos)
        self.create_subscription(String, '/ai_patrol/detections', self._on_detections, 10)

        self.pos = None
        self.nav_state = None
        self.arming_state = None
        self.state = 'INIT'
        self.state_t = time.time()
        self.t_start = None
        self.tick = 0
        self.carrot = None            # 目標点を一定速度で動かす(速度を制御するため)
        self.yaw = float('nan')
        self.targets = []
        self.current = None           # 確認中の Target
        self.bearing = 0.0            # 確認時に候補を見る方位 [rad]
        self.resume_point = None
        self.trail = []
        self.log_events = []
        self.dist_flown = 0.0
        self._last_pos = None

        self.get_logger().info(
            f'捜索ルート: {len(self.route)} 点, 撮影幅 {swath:.1f} m, 巡回高度 {c["patrol_alt"]} m')
        self.create_timer(0.1, self._on_timer)

    # ---- 受信 ---------------------------------------------------------------
    def _on_pos(self, msg):
        self.pos = msg
        if self._last_pos is not None:
            self.dist_flown += math.hypot(msg.x - self._last_pos[0], msg.y - self._last_pos[1])
        self._last_pos = (msg.x, msg.y)

    def _on_status(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def _on_detections(self, msg):
        data = json.loads(msg.data)
        dets = [d for d in data['detections'] if d['label'] in self.cfg['target_labels']]
        if self.state == 'OBSERVE' and self.current is not None:
            self._collect_observation(dets)
        elif self.state in ('PATROL', 'RESUME', 'APPROACH', 'CLIMB'):
            for d in dets:
                if d['conf'] >= self.cfg['candidate_conf']:
                    self._add_candidate(d)

    def _add_candidate(self, d):
        c = self.cfg
        half = c['area_size'] / 2 + 10.0
        if (abs(d['north'] - c['area_center_north']) > half
                or abs(d['east'] - c['area_center_east']) > half):
            return
        for t in self.targets:
            if math.hypot(t.north - d['north'], t.east - d['east']) < c['merge_radius']:
                return
        t = Target(f'T{len(self.targets) + 1}', d['north'], d['east'], d['conf'], d['label'])
        self.targets.append(t)
        self._event(f'{t.id}: 人の候補を発見 (確信度 {d["conf"]:.2f}) '
                    f'位置 北{t.north:.1f} 東{t.east:.1f} m')

    def _collect_observation(self, dets):
        t = self.current
        for d in dets:
            if math.hypot(d['north'] - t.north, d['east'] - t.east) < 5.0:
                t.obs.append((d['north'], d['east'], d['conf']))

    # ---- PX4 送信 -----------------------------------------------------------
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = command, p1, p2
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = self._now_us()
        self.pub_cmd.publish(m)

    def _send(self, n, e, alt, vel=(math.nan, math.nan, math.nan)):
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = not math.isnan(vel[0])
        mode.timestamp = self._now_us()
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = [float(n), float(e), float(-alt)]
        sp.velocity = [float(v) for v in vel]
        sp.yaw = float(self.yaw)
        sp.timestamp = self._now_us()
        self.pub_sp.publish(sp)

    # ---- 目標点の追従(一定速度で目標点を進める)------------------------------
    def _standoff(self, t, alt):
        """候補 t が画像の中央に写る機体の位置(候補の手前 alt*tan(傾き) [m])。"""
        off = look_offset(alt, math.radians(self.cfg['cam_tilt_deg']))
        return t.north - off * math.cos(self.bearing), t.east - off * math.sin(self.bearing)

    def _fly_to(self, n, e, alt, speed, dt=0.1, yaw=None):
        """carrot を (n, e, alt) へ speed [m/s] で近づけて送信。到着したら True。

        yaw を指定しなければ進行方向に機首(=カメラ)を向ける。
        """
        if self.carrot is None:
            self.carrot = [self.pos.x, self.pos.y, -self.pos.z]
        cn, ce, ca = self.carrot
        dn, de = n - cn, e - ce
        dh = math.hypot(dn, de)
        da = alt - ca
        vel = [0.0, 0.0, 0.0]
        step = speed * dt
        if dh > step:
            cn += dn / dh * step
            ce += de / dh * step
            vel[0], vel[1] = dn / dh * speed, de / dh * speed
            if yaw is None and dh > 3.0:
                self.yaw = math.atan2(de, dn)         # 進行方向に機首を向ける
        else:
            cn, ce = n, e
        if yaw is not None:
            self.yaw = yaw
        vstep = self.cfg['vertical_speed'] * dt
        if abs(da) > vstep:
            ca += math.copysign(vstep, da)
            vel[2] = -math.copysign(self.cfg['vertical_speed'], da)
        else:
            ca = alt
        self.carrot = [cn, ce, ca]
        self._send(cn, ce, ca, vel)
        err = math.sqrt((n - self.pos.x) ** 2 + (e - self.pos.y) ** 2 + (alt + self.pos.z) ** 2)
        return (cn, ce, ca) == (n, e, alt) and err < self.cfg['accept_radius']

    # ---- 状態遷移 -----------------------------------------------------------
    def _goto(self, state):
        self.get_logger().info(f'[{self.state} → {state}] {STATE_JP[state]}')
        self.state = state
        self.state_t = time.time()

    def _event(self, text):
        el = 0.0 if self.t_start is None else time.time() - self.t_start
        self.log_events.append(dict(t=round(el, 1), text=text))
        self.get_logger().info(text)

    def _next_pending(self):
        pend = [t for t in self.targets if t.status == 'pending']
        if not pend:
            return None
        return min(pend, key=lambda t: math.hypot(t.north - self.pos.x, t.east - self.pos.y))

    def _on_timer(self):
        self.tick += 1
        if self.pos is None:
            if self.tick % 30 == 0:
                self.get_logger().warn('PX4 の位置情報待ち(SITL と MicroXRCEAgent を確認)')
            return
        c = self.cfg
        if self.state not in ('DONE', 'LAND') and self.tick % 5 == 0:
            self.trail.append((round(self.pos.x, 1), round(self.pos.y, 1)))
        elapsed_state = time.time() - self.state_t

        if self.state == 'INIT':
            self._send(self.pos.x, self.pos.y, -self.pos.z)
            if self.tick > 20:
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self._goto('ARMING')

        elif self.state == 'ARMING':
            self._send(self.pos.x, self.pos.y, -self.pos.z)
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.t_start = time.time()
                self.home = (self.pos.x, self.pos.y)
                self._event('アーム完了。離陸します')
                self._goto('TAKEOFF')
            elif elapsed_state > 3.0:
                self._goto('INIT')

        elif self.state == 'TAKEOFF':
            if self._fly_to(self.home[0], self.home[1], c['patrol_alt'], c['transit_speed']):
                self._event(f'巡回高度 {c["patrol_alt"]:.0f} m に到達。捜索を開始します')
                self._goto('PATROL')

        elif self.state in ('PATROL', 'RESUME'):
            nxt = self._next_pending()
            if nxt is not None:
                self.resume_point = (self.pos.x, self.pos.y)
                self.current = nxt
                nxt.status = 'inspecting'
                self.bearing = math.atan2(nxt.east - self.pos.y, nxt.north - self.pos.x)
                self._event(f'{nxt.id}: 巡回を中断して確認に向かいます')
                self._goto('APPROACH')
            elif self.state == 'RESUME':
                n, e = self.resume_point
                if self._fly_to(n, e, c['patrol_alt'], c['transit_speed']):
                    self._goto('PATROL')
            else:
                n, e = self.route[self.wp]
                if self._fly_to(n, e, c['patrol_alt'], c['patrol_speed']):
                    self.wp += 1
                    if self.wp >= len(self.route):
                        self._event('捜索範囲の巡回を完了。帰還します')
                        self._goto('RETURN')

        elif self.state == 'APPROACH':
            t = self.current
            n, e = self._standoff(t, c['patrol_alt'])
            if self._fly_to(n, e, c['patrol_alt'], c['transit_speed'], yaw=self.bearing):
                self._goto('DESCEND')

        elif self.state == 'DESCEND':
            t = self.current
            n, e = self._standoff(t, c['inspect_alt'])
            if self._fly_to(n, e, c['inspect_alt'], c['patrol_speed'], yaw=self.bearing):
                t.obs = []
                self._goto('OBSERVE')

        elif self.state == 'OBSERVE':
            t = self.current
            n, e = self._standoff(t, c['inspect_alt'])
            self._fly_to(n, e, c['inspect_alt'], 2.0, yaw=self.bearing)
            strong = [o for o in t.obs if o[2] >= c['confirm_conf']]
            if len(strong) >= c['confirm_count'] and elapsed_state >= c['observe_s']:
                # 近距離での推定値の平均を確定位置とする
                t.north = sum(o[0] for o in strong) / len(strong)
                t.east = sum(o[1] for o in strong) / len(strong)
                t.status = 'confirmed'
                lat, lon = ned_to_latlon(t.north, t.east, c['home_lat'], c['home_lon'])
                self._event(f'{t.id}: 要救助者を確定 (最大確信度 {max(o[2] for o in strong):.2f}) '
                            f'緯度 {lat:.6f} 経度 {lon:.6f}')
                self._goto('CLIMB')
            elif elapsed_state >= c['observe_timeout_s']:
                t.status = 'rejected'
                self._event(f'{t.id}: 近距離で確認できず、誤検出として棄却')
                self._goto('CLIMB')
            elif len(t.obs) == 1 and elapsed_state > 1.0:
                # 1回目の近距離観測で位置を補正し、真上に寄せ直す
                t.north, t.east = t.obs[0][0], t.obs[0][1]

        elif self.state == 'CLIMB':
            t = self.current
            n, e = self._standoff(t, c['inspect_alt'])
            if self._fly_to(n, e, c['patrol_alt'], c['patrol_speed'], yaw=self.bearing):
                self.current = None
                self._goto('RESUME')

        elif self.state == 'RETURN':
            if self._fly_to(self.home[0], self.home[1], c['patrol_alt'], c['transit_speed']):
                self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._event('離陸地点の上空に到着。着陸します')
                self._goto('LAND')

        elif self.state == 'LAND':
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self._event('着陸完了。ミッション終了')
                self._write_report()
                self._goto('DONE')

        if self.tick % 2 == 0:
            self._publish_state()

    # ---- 出力 -------------------------------------------------------------
    def _summary(self):
        c = self.cfg
        return dict(
            state=self.state, state_jp=STATE_JP[self.state],
            elapsed=0.0 if self.t_start is None else round(time.time() - self.t_start, 1),
            drone=dict(north=self.pos.x, east=self.pos.y, alt=-self.pos.z),
            yaw=self.yaw, wp=self.wp, route=self.route,
            area=dict(n=c['area_center_north'], e=c['area_center_east'], size=c['area_size']),
            patrol_alt=c['patrol_alt'], inspect_alt=c['inspect_alt'],
            current=None if self.current is None else self.current.id,
            targets=[t.as_dict(c['home_lat'], c['home_lon']) for t in self.targets],
            events=self.log_events[-8:], dist_flown=round(self.dist_flown, 1),
        )

    def _publish_state(self):
        s = self._summary()
        s['trail'] = self.trail[-2000:]
        self.pub_state.publish(String(data=json.dumps(s, ensure_ascii=False)))

    def _write_report(self):
        os.makedirs(self.cfg['report_dir'], exist_ok=True)
        path = os.path.join(self.cfg['report_dir'],
                            f'mission_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        rep = self._summary()
        # 報告書には環境依存の保存先パスを残さない
        cfg = {k: v for k, v in self.cfg.items() if k != 'report_dir'}
        rep.update(trail=self.trail, events=self.log_events, config=cfg)
        with open(path, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f'報告書を保存: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
