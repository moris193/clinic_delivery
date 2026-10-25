"""
ミッション結果のグラフ化（段階7）

使い方:
  ros2 run clinic_delivery plot_mission                 # 最新の結果
  ros2 run clinic_delivery plot_mission <結果フォルダ>

出力（結果フォルダ内）:
  trajectory.png … 地図の上にロボットの走行軌跡を描いた図
  timeline.png   … 時間ごとにどの部屋へ向かっていたかの図
"""
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

from clinic_delivery.stations import STATIONS  # noqa: E402

NAMES_EN = {'reception': 'Reception', 'exam': 'Exam room',
            'treatment': 'Treatment room', 'lab': 'Lab'}
COLORS = {'reception': '#3399ff', 'exam': '#33cc4d',
          'treatment': '#ffb31a', 'lab': '#e63333'}


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    parts = data.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    px = parts[3]
    return [[px[r * w + c] for c in range(w)] for r in range(h)], w, h


def main():
    base = '/workspace/ros2_ws/results'
    if len(sys.argv) > 1:
        d = sys.argv[1]
    else:
        dirs = sorted(glob.glob(os.path.join(base, 'mission_*')))
        if not dirs:
            print('結果フォルダが見つかりません')
            return
        d = dirs[-1]

    traj = list(csv.DictReader(open(os.path.join(d, 'trajectory.csv'), encoding='utf-8-sig')))
    events = list(csv.DictReader(open(os.path.join(d, 'events.csv'), encoding='utf-8-sig')))
    t = [float(r['sim_time_s']) for r in traj]
    xs = [float(r['x']) for r in traj]
    ys = [float(r['y']) for r in traj]

    # ---- 軌跡 ----
    share = get_package_share_directory('clinic_delivery')
    img, w, h = read_pgm(os.path.join(share, 'maps', 'clinic_map.pgm'))
    res, ox, oy = 0.05, -4.5, -3.5
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.imshow(img, cmap='gray', vmin=0, vmax=255,
              extent=[ox, ox + w * res, oy, oy + h * res])
    sc = ax.scatter(xs, ys, c=t, cmap='viridis', s=4)
    for k, (x, y, _) in STATIONS.items():
        ax.scatter([x], [y], s=250, c=COLORS[k], edgecolors='black', zorder=3)
        ax.annotate(NAMES_EN[k], (x, y), xytext=(-30, 14), textcoords='offset points',
                    fontsize=10, weight='bold')
    fig.colorbar(sc, ax=ax, label='Simulation time [s]')
    total = t[-1] if t else 0
    ax.set_title(f'Delivery robot trajectory ({total:.0f} s = {total / 60:.1f} min)')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'trajectory.png'), dpi=150)

    # ---- タイムライン ----
    fig, ax = plt.subplots(figsize=(11, 3.5))
    rows = list(STATIONS)
    start = None
    for e in events:
        tt = float(e['sim_time_s'])
        if e['event'] in ('出発',):
            start = (tt, e['station'])
        elif e['event'] in ('到着', '中断', '失敗') and start:
            s0, st = start
            y = rows.index(st)
            hatch = '//' if e['event'] != '到着' else None
            ax.barh(y, tt - s0, left=s0, color=COLORS[st], hatch=hatch,
                    edgecolor='black', height=0.6)
            start = None
        elif e['event'] == '緊急':
            ax.axvline(tt, color='red', linestyle='--')
            ax.text(tt, len(rows) - 0.75, ' EMERGENCY', color='red', fontsize=9)
        elif e['event'] == '終了':
            ax.axvline(tt, color='black', linestyle=':')
            ax.text(tt, len(rows) - 0.75, ' time up', fontsize=9)
    ax.set_ylim(-0.6, len(rows) - 0.2)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([NAMES_EN[r] for r in rows])
    ax.set_xlabel('Simulation time [s]')
    ax.set_title('Delivery timeline (bar = travelling to the room, hatched = interrupted)')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'timeline.png'), dpi=150)

    print(f'保存しました: {d}/trajectory.png, {d}/timeline.png')


if __name__ == '__main__':
    main()
