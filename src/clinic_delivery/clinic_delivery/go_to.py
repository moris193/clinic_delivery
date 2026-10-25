"""
指定した部屋へロボットを自律走行させる動作確認用スクリプト（段階5）

使い方:
  ros2 run clinic_delivery go_to exam        # 診察室へ
  ros2 run clinic_delivery go_to treatment   # 処置室へ
  ros2 run clinic_delivery go_to lab         # 検査室へ
  ros2 run clinic_delivery go_to reception   # 受付へ
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter

from clinic_delivery.stations import LABELS, STATIONS


def make_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # 時刻は 0 のまま（＝最新のTFを使う）。シミュレーション時刻とのずれを避けるため
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in STATIONS:
        print('行き先を指定してください: ' + ', '.join(STATIONS))
        return

    name = sys.argv[1]
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    # AMCL の初期位置は設定ファイルで与えているので、ここでは再設定しない
    nav.waitUntilNav2Active(localizer='map_server')

    x, y, yaw = STATIONS[name]
    print(f'{LABELS[name]} ({x}, {y}) へ向かいます')
    nav.goToPose(make_pose(x, y, yaw))

    i = 0
    while not nav.isTaskComplete():
        i += 1
        fb = nav.getFeedback()
        if fb and i % 20 == 0:
            print(f'  残り距離 {fb.distance_remaining:.2f} m')
        rclpy.spin_once(nav, timeout_sec=0.1)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'{LABELS[name]} に到着しました')
    elif result == TaskResult.CANCELED:
        print('キャンセルされました')
    else:
        print('到着できませんでした')

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
