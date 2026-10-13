#!/usr/bin/env python3
"""ミッション報告書 JSON を正解データ(worlds/targets.json)と照合して評価する。

出力: 検出数・見逃し・誤検出・位置誤差の表(標準出力)と、結果地図 PNG。
実行: python3 evaluate_mission.py results/mission_XXXX.json [--match 5.0]
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
for name in ('Noto Sans CJK JP', 'IPAexGothic', 'Hiragino Sans'):
    if any(name in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = name
        break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('report')
    ap.add_argument('--targets', default=str(ROOT / 'worlds/targets.json'))
    ap.add_argument('--match', type=float, default=5.0, help='正解と対応づける距離 [m]')
    a = ap.parse_args()

    rep = json.loads(Path(a.report).read_text())
    gt = json.loads(Path(a.targets).read_text())
    persons = gt['persons']
    confirmed = [t for t in rep['targets'] if t['status'] == 'confirmed']
    rejected = [t for t in rep['targets'] if t['status'] == 'rejected']

    # 確定した対象を最も近い正解に対応づける(貪欲法)
    pairs, used = [], set()
    cand = sorted(((math.hypot(t['north'] - p['north'], t['east'] - p['east']), i, j)
                   for i, t in enumerate(confirmed) for j, p in enumerate(persons)))
    done_t = set()
    for d, i, j in cand:
        if d <= a.match and i not in done_t and j not in used:
            pairs.append((i, j, d))
            done_t.add(i)
            used.add(j)
    tp = len(pairs)
    fp = len(confirmed) - tp
    fn = len(persons) - tp
    # 巡回高度での初期推定の誤差(確認降下で何 m 改善したか)
    first_err = [math.hypot(confirmed[i]['first_north'] - persons[j]['north'],
                            confirmed[i]['first_east'] - persons[j]['east']) for i, j, _ in pairs]
    final_err = [d for _, _, d in pairs]
    # 棄却した候補が本当に人ではなかったか
    rej_true = sum(any(math.hypot(t['north'] - p['north'], t['east'] - p['east']) <= a.match
                       for p in persons) for t in rejected)

    print(f'正解の人数            : {len(persons)}')
    print(f'確定 (うち正解 / 誤り) : {len(confirmed)} ({tp} / {fp})')
    print(f'見逃し                : {fn}  {[persons[j]["id"] for j in range(len(persons)) if j not in used]}')
    print(f'棄却した候補          : {len(rejected)} (うち実は人だった: {rej_true})')
    print(f'再現率 {tp / len(persons):.0%}   適合率 {tp / max(1, len(confirmed)):.0%}')
    if pairs:
        print(f'位置誤差 巡回高度での推定: 平均 {sum(first_err) / len(first_err):.2f} m'
              f' / 確認後: 平均 {sum(final_err) / len(final_err):.2f} m (最大 {max(final_err):.2f} m)')
        for i, j, d in pairs:
            print(f'  {persons[j]["id"]} ({persons[j]["note"]}) ← {confirmed[i]["id"]}  誤差 {d:.2f} m'
                  f'  緯度 {confirmed[i]["lat"]:.6f} 経度 {confirmed[i]["lon"]:.6f}')
    print(f'飛行時間 {rep["elapsed"]:.0f} s   飛行距離 {rep["dist_flown"]:.0f} m')

    # ---- 地図 --------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 7.5))
    ar = rep['area']
    h = ar['size'] / 2
    ax.add_patch(plt.Rectangle((ar['e'] - h, ar['n'] - h), ar['size'], ar['size'],
                               fill=False, ec='green', lw=1.5, label='捜索範囲'))
    r = rep['route']
    ax.plot([p[1] for p in r], [p[0] for p in r], '--', c='gray', lw=0.8, label='予定ルート')
    tr = rep['trail']
    ax.plot([p[1] for p in tr], [p[0] for p in tr], c='orange', lw=1.5, label='飛行軌跡')
    ax.scatter([p['east'] for p in persons], [p['north'] for p in persons], s=160,
               facecolors='none', edgecolors='blue', lw=1.5, label='正解(人)')
    for p in persons:
        ax.annotate(p['id'], (p['east'], p['north']), xytext=(6, 6), textcoords='offset points',
                    color='blue', fontsize=9)
    ax.scatter([t['east'] for t in confirmed], [t['north'] for t in confirmed], marker='o',
               c='red', s=40, label='AI が確定')
    ax.scatter([t['first_east'] for t in confirmed], [t['first_north'] for t in confirmed],
               marker='+', c='red', s=50, label='巡回高度での初期推定')
    if rejected:
        ax.scatter([t['east'] for t in rejected], [t['north'] for t in rejected], marker='x',
                   c='gray', s=50, label='誤検出として棄却')
    ax.plot(0, 0, 's', c='k', ms=8, label='離陸地点')
    ax.set_xlabel('東 [m]')
    ax.set_ylabel('北 [m]')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_title(f'捜索結果: 正解 {len(persons)} 名中 {tp} 名を発見(誤り {fp} 件)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=3, fontsize=8)
    fig.tight_layout()
    out = Path(a.report).with_suffix('.png')
    fig.savefig(out, dpi=150)
    print('地図:', out)


if __name__ == '__main__':
    main()
