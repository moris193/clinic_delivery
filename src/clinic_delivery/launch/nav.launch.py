"""
診療所シミュレーション + Nav2 を一括起動する launch ファイル（段階5）

起動するもの:
  1. sim.launch.py（Gazebo、ブリッジ、LiDARのTF）
  2. 地図（maps/clinic_map.yaml）と自己位置推定
       use_amcl:=true  … AMCL（LiDARと地図を照合して位置を推定）【標準】
       use_amcl:=false … 推定せず、map と odom を固定でつなぐ（保険）
  3. Nav2 本体（経路計画・追従）… Gazebo 起動を待つため 10 秒後に開始
  4. RViz2（Nav2 用の表示設定）

使い方:
  ros2 launch clinic_delivery nav.launch.py
  ros2 launch clinic_delivery nav.launch.py gui:=false       # Gazebo画面なし（軽い）
  ros2 launch clinic_delivery nav.launch.py use_amcl:=false  # 位置推定なし
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('clinic_delivery')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_share, 'maps', 'clinic_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_file = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    use_amcl = LaunchConfiguration('use_amcl')
    gui = LaunchConfiguration('gui')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={'rviz': 'false', 'gui': gui}.items())

    # --- 自己位置推定：AMCL（標準）---
    localization_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_file,
                          'params_file': params_file,
                          'use_sim_time': 'true',
                          'autostart': 'true'}.items(),
        condition=IfCondition(use_amcl))

    # --- 自己位置推定なし（保険）：地図だけ出して map->odom を固定 ---
    # ロボットは受付 (x=-2.5, y=0) から出発するので、その分ずらしてつなぐ
    localization_static = GroupAction(
        condition=UnlessCondition(use_amcl),
        actions=[
            Node(package='nav2_map_server', executable='map_server',
                 name='map_server', output='screen',
                 parameters=[params_file, {'yaml_filename': map_file,
                                           'use_sim_time': True}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['map_server']}]),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--x', '-2.5', '--y', '0', '--z', '0',
                            '--frame-id', 'map', '--child-frame-id', 'odom'],
                 output='screen'),
        ])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': params_file,
                          'use_sim_time': 'true',
                          'autostart': 'true'}.items())

    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': True}],
                output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_amcl', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        sim,
        TimerAction(period=10.0, actions=[
            localization_amcl, localization_static, navigation, rviz]),
    ])
