#!/usr/bin/env python3
"""カメラ傾きテストの画像で、人の検出数と確信度を比べる(正解は各画像 5 人)。

実行: python3 eval_tilt.py FRAME_DIR MODEL.onnx
"""
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/ai_patrol'))
from ai_patrol.yolo_detector import YoloDetector, draw_detections  # noqa: E402

frame_dir = Path(sys.argv[1])
det = YoloDetector(sys.argv[2], conf_th=0.25,
                   providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
out = frame_dir / 'annotated'
out.mkdir(exist_ok=True)
print(f'{"条件":>14} {"分割":>4} {"人":>3}  確信度')
for p in sorted(frame_dir.glob('cam_*.png')):
    img = cv2.imread(str(p))
    for tiles in [(2, 2), (3, 3)]:
        ds = [d for d in det.detect(img, tiles=tiles) if d.label == 'person']
        confs = sorted((d.conf for d in ds), reverse=True)
        print(f'{p.stem[4:]:>14} {tiles[0]}x{tiles[1]:<2} {len(ds):>3}  ' + ' '.join(f'{c:.2f}' for c in confs))
        cv2.imwrite(str(out / f'{p.stem}_{tiles[0]}x{tiles[1]}.jpg'), draw_detections(img.copy(), ds))
