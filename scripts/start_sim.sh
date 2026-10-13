#!/usr/bin/env bash
# ターミナル1: PX4 SITL + Gazebo(画面つき)を捜索ワールド・捜索用機体で起動する
source "$(dirname "$0")/env.sh"
cd "$PX4_DIR"
# PX4 のビルドは ~/.local の kconfiglib などを使うので、ここでは無効化しない
unset PYTHONNOUSERSITE
# ノート PC のハイブリッド GPU で、Gazebo の描画を NVIDIA 側で行う
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
# 機体: 捜索用機体(エアフレーム 4099, gz_models/)、ワールド: okinawa_sar(worlds/)
# Gazebo の画面は start_view.sh で別に開く(パネルなし・ドローン追従)。標準の画面が良ければ
# HEADLESS= ./start_sim.sh のように空にして実行する
export HEADLESS=${HEADLESS-1}
make px4_sitl gz_x500_sar_cam_okinawa_sar
