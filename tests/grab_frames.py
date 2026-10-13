#!/usr/bin/env python3
"""gz-transport で複数カメラの画像を1枚ずつ保存する(高度別テスト用)。

実行: PYTHONNOUSERSITE=1 python3 grab_frames.py OUT_DIR [WORLD CAM1 CAM2 ...]
  WORLD を省略すると高度別テスト(alt_test, cam_10m ... cam_30m)
"""
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

out = Path(sys.argv[1] if len(sys.argv) > 1 else 'frames')
if len(sys.argv) > 3:
    WORLD, CAMS = sys.argv[2], sys.argv[3:]
else:
    WORLD, CAMS = 'alt_test', [f'cam_{a}m' for a in (10, 15, 20, 25, 30)]
out.mkdir(parents=True, exist_ok=True)
got = {}
lock = threading.Lock()


def make_cb(cam):
    def cb(msg: Image):
        with lock:
            if cam in got:
                return
            arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            name = cam.replace('cam_', 'alt_') if WORLD == 'alt_test' else cam
            PILImage.fromarray(arr).save(out / f'{name}.png')
            got[cam] = True
            print('saved', cam, flush=True)
    return cb


node = Node()
for c in CAMS:
    topic = f'/world/{WORLD}/model/{c}/link/camera_link/sensor/camera/image'
    node.subscribe(Image, topic, make_cb(c))

t0 = time.time()
while len(got) < len(CAMS) and time.time() - t0 < 120:
    time.sleep(0.5)
print('done', sorted(got))
