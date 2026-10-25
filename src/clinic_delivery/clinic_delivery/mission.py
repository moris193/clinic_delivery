"""
院内搬送ミッション（段階6）

受付を起点に、診療所の業務フローに沿って物品を巡回搬送する。
  1. 診察室へカルテ・書類を届ける
  2. 検査室へ検体を届ける
  3. 処置室へ処置物品を届ける
  4. 受付へ戻る
各地点では受け渡しを模擬して数秒停車する。

緊急検体の割り込み:
  /emergency トピックに部屋の名前が届くと、今の移動を中断してその部屋へ
  最優先で向かい、終わったら中断した搬送を再開する。
    ros2 topic pub --once /emergency std_msgs/msg/String "{data: lab}"
  決まった時刻に自動で割り込ませることもできる（再現性のある実験用）:
    -p emergency_at:=150.0 -p emergency_station:=lab

6分間の運用:
  シミュレーション時間で duration 秒（標準 360 秒）経過したら、搬送を打ち切って
  受付へ帰還し、結果を保存して終了する。

結果の保存先: /workspace/ros2_ws/results/mission_日時/
  events.csv      … 出発・到着・停車・割り込みなどの記録
  trajectory.csv  … ロボットの位置（0.5秒ごと）
  summary.txt     … 搬送件数、走行距離、経過時間などのまとめ

使い方:
  ros2 run clinic_delivery mission
  ros2 run clinic_delivery mission --ros-args -p emergency_at:=150.0
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
import rclpy.time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from clinic_delivery.go_to import make_pose
from clinic_delivery.stations import LABELS, STATIONS

# 1巡分の搬送タスク（行き先, 内容）
ROUTE = [
    ('exam', 'カルテ・書類の配送'),
    ('lab', '検体の配送'),
    ('treatment', '処置物品の配送'),
    ('reception', '受付へ帰着'),
]


class Mission:
    def __init__(self):
        self.nav = BasicNavigator(node_name='clinic_mission')
        self.nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        n = self.nav
        n.declare_parameter('duration', 360.0)
        n.declare_parameter('dwell', 3.0)
        n.declare_parameter('emergency_at', -1.0)
        n.declare_parameter('emergency_station', 'lab')
        n.declare_parameter('result_dir', '/workspace/ros2_ws/results')
        self.duration = n.get_parameter('duration').value
        self.dwell = n.get_parameter('dwell').value
        self.emergency_at = n.get_parameter('emergency_at').value
        self.emergency_station = n.get_parameter('emergency_station').value

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.out_dir = os.path.join(n.get_parameter('result_dir').value,
                                    'mission_' + stamp)
        os.makedirs(self.out_dir, exist_ok=True)
        self.ev_file = open(os.path.join(self.out_dir, 'events.csv'), 'w', newline='', encoding='utf-8-sig')
        self.ev = csv.writer(self.ev_file)
        self.ev.writerow(['sim_time_s', 'event', 'station', 'detail'])
        self.tr_file = open(os.path.join(self.out_dir, 'trajectory.csv'), 'w', newline='', encoding='utf-8-sig')
        self.tr = csv.writer(self.tr_file)
        self.tr.writerow(['sim_time_s', 'x', 'y'])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, n)
        n.create_subscription(String, 'emergency', self.on_emergency, 10)
        n.create_timer(0.5, self.record_pose)

        self.pending_emergency = None
        self.auto_emergency_done = False
        self.t0 = None
        self.last_xy = None
        self.distance = 0.0
        self.delivered = {k: 0 for k in STATIONS}
        self.emergencies = 0
        self.failures = 0
        self.legs = []  # (行き先, 所要時間)
        self.wall0 = time.time()

    # ---------- 共通 ----------
    def now(self):
        return self.nav.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.now() - self.t0 if self.t0 is not None else 0.0

    def log(self, event, station='', detail=''):
        t = self.elapsed()
        self.ev.writerow([f'{t:.1f}', event, station, detail])
        self.ev_file.flush()
        label = LABELS.get(station, station)
        print(f'[{t:6.1f} s] {event} {label} {detail}', flush=True)

    def spin(self, sec=0.1):
        rclpy.spin_once(self.nav, timeout_sec=sec)

    def time_up(self):
        return self.elapsed() >= self.duration

    # ---------- コールバック ----------
    def on_emergency(self, msg):
        name = msg.data.strip()
        if name in STATIONS:
            self.pending_emergency = name
        else:
            print(f'不明な部屋名です: {name}（{", ".join(STATIONS)}）')

    def record_pose(self):
        if self.t0 is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        if self.last_xy is not None:
            d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if d < 1.0:  # 位置推定の飛びは距離に入れない
                self.distance += d
        self.last_xy = (x, y)
        self.tr.writerow([f'{self.elapsed():.1f}', f'{x:.3f}', f'{y:.3f}'])

    def check_auto_emergency(self):
        if (self.emergency_at >= 0 and not self.auto_emergency_done
                and self.elapsed() >= self.emergency_at):
            self.auto_emergency_done = True
            self.pending_emergency = self.emergency_station

    # ---------- 移動 ----------
    def go(self, station, detail, interruptible=True):
        """station へ移動。結果: 'ok' / 'interrupted' / 'timeup' / 'failed'"""
        x, y, yaw = STATIONS[station]
        start = self.elapsed()
        self.log('出発', station, detail)
        if not self.nav.goToPose(make_pose(x, y, yaw)):
            self.log('失敗', station, '目標が受け付けられませんでした')
            return 'failed'

        while not self.nav.isTaskComplete():
            self.spin()
            self.check_auto_emergency()
            if interruptible and self.pending_emergency and self.pending_emergency != station:
                self.nav.cancelTask()
                while not self.nav.isTaskComplete():
                    self.spin()
                self.log('中断', station, '緊急検体のため')
                return 'interrupted'
            if interruptible and self.time_up():
                self.nav.cancelTask()
                while not self.nav.isTaskComplete():
                    self.spin()
                self.log('中断', station, '運用時間終了のため')
                return 'timeup'

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.legs.append((station, self.elapsed() - start))
            self.log('到着', station, f'所要 {self.elapsed() - start:.1f} 秒')
            return 'ok'
        self.failures += 1
        self.log('失敗', station, '到着できませんでした')
        return 'failed'

    def stop_for(self, station, detail):
        self.log('停車', station, detail)
        end = self.elapsed() + self.dwell
        while self.elapsed() < end:
            self.spin()
            self.check_auto_emergency()

    def handle_emergency(self):
        station = self.pending_emergency
        self.pending_emergency = None
        self.emergencies += 1
        self.log('緊急', station, '緊急検体の対応を開始')
        if self.go(station, '緊急検体', interruptible=False) == 'ok':
            self.stop_for(station, '緊急検体の受け渡し')
            self.log('緊急完了', station)

    # ---------- 全体 ----------
    def run(self):
        print('Nav2 の起動を待っています…')
        self.nav.waitUntilNav2Active(localizer='map_server')
        while self.now() == 0.0:  # /clock 待ち
            self.spin()
        self.t0 = self.now()
        self.wall0 = time.time()
        print(f'ミッション開始（運用時間 {self.duration:.0f} 秒）')
        self.log('開始', 'reception')

        i = 0
        while not self.time_up():
            if self.pending_emergency:
                self.handle_emergency()
                continue
            station, detail = ROUTE[i % len(ROUTE)]
            r = self.go(station, detail)
            if r == 'ok':
                if station != 'reception':
                    self.delivered[station] += 1
                self.stop_for(station, '受け渡し')
                i += 1
            elif r == 'failed':
                i += 1  # 失敗した地点は飛ばして次へ
            # interrupted → 次のループで緊急対応してから同じ地点へ再挑戦
            # timeup → ループを抜ける

        self.log('終了', '', f'運用時間 {self.duration:.0f} 秒に到達。受付へ帰還')
        self.go('reception', '帰還', interruptible=False)
        self.finish()

    def finish(self):
        sim_total = self.elapsed()
        wall_total = time.time() - self.wall0
        total = sum(self.delivered.values())
        lines = [
            '=== 院内搬送ミッション 結果 ===',
            f'運用時間（設定）        : {self.duration:.0f} 秒',
            f'総経過時間（シミュレーション）: {sim_total:.1f} 秒（{sim_total / 60:.1f} 分）',
            f'総経過時間（実時間）    : {wall_total:.1f} 秒',
            f'実時間比（RTF）         : {sim_total / wall_total:.2f}',
            f'搬送件数 合計           : {total} 件',
        ]
        for k in ('exam', 'lab', 'treatment'):
            lines.append(f'  {LABELS[k]:<6}         : {self.delivered[k]} 件')
        lines += [
            f'緊急対応                : {self.emergencies} 件',
            f'到着失敗                : {self.failures} 件',
            f'走行距離                : {self.distance:.1f} m',
        ]
        if self.legs:
            avg = sum(t for _, t in self.legs) / len(self.legs)
            lines.append(f'1区間の平均所要時間     : {avg:.1f} 秒')
        text = '\n'.join(lines)
        print('\n' + text)
        with open(os.path.join(self.out_dir, 'summary.txt'), 'w', encoding='utf-8-sig') as f:
            f.write(text + '\n')
        self.ev_file.close()
        self.tr_file.close()
        print(f'\n結果を保存しました: {self.out_dir}')


def main():
    rclpy.init()
    m = Mission()
    try:
        m.run()
    except KeyboardInterrupt:
        print('\n中止しました。ここまでの結果を保存します')
        try:
            m.nav.cancelTask()
        except Exception:
            pass
        m.finish()
    try:
        m.nav.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
