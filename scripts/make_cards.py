#!/usr/bin/env python3
"""動画用のタイトル画面・まとめ画面・「2倍速」表示を PNG で作る(ffmpeg に drawtext が無いため)。

実行: python3 make_cards.py OUT_DIR REPORT.json [MAP.png]
"""
import json
import math
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
out = Path(sys.argv[1])
rep = json.loads(Path(sys.argv[2]).read_text())
map_png = Path(sys.argv[3]) if len(sys.argv) > 3 else None
out.mkdir(parents=True, exist_ok=True)
fp = subprocess.run(['fc-match', '-f', '%{file}', 'Noto Sans CJK JP:bold'],
                    capture_output=True, text=True).stdout.strip()
F = {s: ImageFont.truetype(fp, s) for s in (28, 36, 44, 64, 84)}
BG, TX, SUB, ACC = (24, 26, 32), (240, 240, 240), (170, 175, 185), (255, 120, 80)


def center(d, y, text, font, fill):
    w = d.textlength(text, font=font)
    d.text(((W - w) / 2, y), text, font=font, fill=fill)


# ---- タイトル --------------------------------------------------------------
im = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(im)
center(d, 250, 'AI 捜索ドローン', F[84], TX)
center(d, 380, '災害時の要救助者を、AI で見つけて・確かめて・位置を知らせる', F[44], ACC)
items = ['PX4 SITL + Gazebo Harmonic + ROS 2 Humble(オフボード制御)',
         'YOLO11m(ONNX, GPU 推論)× タイル分割推論',
         '前方斜め下 30° カメラ + 2 段階確認(巡回で発見 → 降下して再確認)',
         '舞台: 沖縄県内の学校周辺(120 m 四方)に要救助者 6 名を配置']
for i, t in enumerate(items):
    center(d, 520 + i * 70, t, F[36], SUB)
center(d, 900, '動画は 2 倍速で再生しています', F[36], TX)
im.save(out / 'title.png')

# ---- まとめ ------------------------------------------------------------------
im = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(im)
d.text((80, 60), '結果', font=F[64], fill=TX)
conf = [t for t in rep['targets'] if t['status'] == 'confirmed']
rej = [t for t in rep['targets'] if t['status'] == 'rejected']
lines = [
    (f'要救助者 {len(conf)} 名を確定(正解 6 名)', F[44], ACC),
    (f'誤検出の候補 {len(rej)} 件は、降下して再確認し棄却', F[36], TX),
    (f'飛行時間 {rep["elapsed"]:.0f} 秒 / 飛行距離 {rep["dist_flown"]:.0f} m', F[36], TX),
    ('確定位置は緯度・経度で記録(救助隊への通報を想定)', F[36], TX),
]
y = 190
for t, f, c in lines:
    d.text((80, y), t, font=f, fill=c)
    y += 80
y += 20
for t in conf:
    d.text((110, y), f'{t["id"]}  緯度 {t["lat"]:.6f}  経度 {t["lon"]:.6f}', font=F[28], fill=SUB)
    y += 44
d.text((80, 980), 'コード: GitHub(説明欄)  /  PAI 最終課題', font=F[28], fill=SUB)
if map_png and map_png.exists():
    m = Image.open(map_png).convert('RGB')
    r = min(900 / m.height, 820 / m.width)
    m = m.resize((int(m.width * r), int(m.height * r)))
    im.paste(m, (W - m.width - 60, (H - m.height) // 2))
im.save(out / 'summary.png')

# ---- 2倍速バッジ(透過 PNG)----------------------------------------------------
b = Image.new('RGBA', (170, 56), (0, 0, 0, 0))
d = ImageDraw.Draw(b)
d.rounded_rectangle((0, 0, 169, 55), 12, fill=(220, 60, 40, 230))
d.text((22, 6), '2倍速', font=F[36], fill=(255, 255, 255, 255))
b.save(out / 'speed_badge.png')
print('cards:', out)
