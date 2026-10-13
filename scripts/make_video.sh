#!/usr/bin/env bash
# 録画した生動画から提出用の動画(1920x1080, 2倍速)を作る
#   使い方: make_video.sh RAW.mp4 REPORT.json [OUT.mp4]
#   構成: タイトル(5秒)→ 本編(2倍速。表示画面の右上に Gazebo の 3D 画面を合成)→ 結果(8秒)
set -e
source "$(dirname "$0")/env.sh"
FF=$(command -v ffmpeg || echo "$HOME/anaconda3/bin/ffmpeg")
RAW=$1; REP=$2; OUT=${3:-$PAI_ROOT/video/PAI_final_demo.mp4}
TMP=$(mktemp -d)
MAP=${REP%.json}.png
[ -f "$MAP" ] || /usr/bin/python3 "$PAI_ROOT/tests/evaluate_mission.py" "$REP" > /dev/null
/usr/bin/python3 "$(dirname "$0")/make_cards.py" "$TMP" "$REP" "$MAP"

ENC="-c:v h264_nvenc -preset p5 -cq 21 -pix_fmt yuv420p -r 30"
# 本編: 左モニタ(表示画面)を土台に、右モニタの Gazebo 3D ビュー部分(タイトルバーを除く)を
# 縮小して 3D 枠に重ね、2倍速にする
"$FF" -loglevel error -y -i "$RAW" -i "$TMP/speed_badge.png" -filter_complex "\
  [0:v]crop=2048:1152:0:0,scale=1920:1080[base];\
  [0:v]crop=1770:996:2123:156,scale=860:484[view];\
  [base][view]overlay=1040:70[v1];\
  [v1][1:v]overlay=1730:12,setpts=PTS/2[v]" -map "[v]" -an $ENC "$TMP/main.mp4"
"$FF" -loglevel error -y -loop 1 -t 5 -i "$TMP/title.png" $ENC "$TMP/title.mp4"
"$FF" -loglevel error -y -loop 1 -t 8 -i "$TMP/summary.png" $ENC "$TMP/summary.mp4"
printf "file '%s'\n" "$TMP/title.mp4" "$TMP/main.mp4" "$TMP/summary.mp4" > "$TMP/list.txt"
"$FF" -loglevel error -y -f concat -safe 0 -i "$TMP/list.txt" -c copy "$OUT"
echo "完成: $OUT ($("$FF" -i "$OUT" 2>&1 | grep -o 'Duration: [0-9:.]*'))"
