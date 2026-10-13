#!/usr/bin/env bash
# ターミナル2: DDS エージェント + AI 捜索ノード一式を起動する(start_sim.sh の後で)
source "$(dirname "$0")/env.sh"
source /opt/ros/humble/setup.bash
source "$PX4_WS/install/setup.bash"
source "$PAI_ROOT/ros2_ws/install/setup.bash"
# 仮想環境の onnxruntime-gpu と NumPy 1.26 を優先して読み込む(ノードはシステムの python3 で動く)
export PYTHONPATH=$(ls -d "$PAI_VENV"/lib/python3.*/site-packages):$PYTHONPATH
# pip で入れた CUDA / cuDNN ライブラリを onnxruntime-gpu から見えるようにする
NV=$(ls -d "$PAI_VENV"/lib/python3.*/site-packages/nvidia/*/lib 2>/dev/null | paste -sd:)
export LD_LIBRARY_PATH=$NV:$LD_LIBRARY_PATH

pgrep -x MicroXRCEAgent > /dev/null || (MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1 &)
sleep 2
ros2 launch ai_patrol ai_patrol.launch.py \
  model:="$PAI_MODELS/yolo11m.onnx" cpu_model:="$PAI_MODELS/yolo11s.onnx" \
  report_dir:="$PAI_ROOT/results" "$@"
