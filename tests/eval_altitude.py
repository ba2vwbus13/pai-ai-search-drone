#!/usr/bin/env python3
"""高度別画像に対して、全体推論とタイル推論の検出数を比べる。

実行: python3 eval_altitude.py FRAME_DIR MODEL.onnx
各高度の正解は 人3(立位2・倒れている1)・車2。
"""
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/ai_patrol'))
from ai_patrol.yolo_detector import YoloDetector, draw_detections  # noqa: E402

frame_dir = Path(sys.argv[1])
model = sys.argv[2]
det = YoloDetector(model, conf_th=0.25)

print(f'{"高度":>4} {"方式":>6} {"人":>3} {"車":>3} {"時間ms":>7}')
for p in sorted(frame_dir.glob('alt_*.png')):
    img = cv2.imread(str(p))
    for name, tiles in [('全体', (1, 1)), ('2x2', (2, 2)), ('3x3', (3, 3))]:
        t0 = time.time()
        ds = det.detect(img, tiles=tiles)
        ms = (time.time() - t0) * 1000
        n_p = sum(d.label == 'person' for d in ds)
        n_v = sum(d.label == 'vehicle' for d in ds)
        print(f'{p.stem[4:]:>5} {name:>6} {n_p:>3} {n_v:>3} {ms:7.0f}  '
              + ' '.join(f'{d.coco}:{d.conf:.2f}' for d in ds))
        out = frame_dir / 'annotated'
        out.mkdir(exist_ok=True)
        cv2.imwrite(str(out / f'{p.stem}_{name}.jpg'), draw_detections(img.copy(), ds))
