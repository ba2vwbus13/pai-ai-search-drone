#!/usr/bin/env bash
# Gazebo の 3D 画面(デモ動画用)を開き、カメラでドローンを追従する
# start_sim.sh が HEADLESS で Gazebo サーバーを起動した後に実行する
source "$(dirname "$0")/env.sh"
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
gz sim -g --gui-config "$PAI_ROOT/gz_models/gui_follow.config" > /tmp/pai_gui.log 2>&1 &
sleep 8
# ドローンの後方上空からドローンを注視したまま追従する(FOLLOW_LOOK_AT)。
# offset は機体基準の [m](前・左・上)。後方 9m・上 9m から見下ろして追いかける、pgain は追従の速さ
gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: FOLLOW_LOOK_AT,
  follow_target: {name: "x500_sar_cam_0"}, track_target: {name: "x500_sar_cam_0"},
  follow_offset: {x: -9, y: 0, z: 9}, follow_pgain: 0.2, track_pgain: 0.5'
echo "3D 画面: x500_sar_cam_0 を追従中"
wait
