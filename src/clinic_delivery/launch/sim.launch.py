"""
診療所ワールドと搬送ロボットを一括起動する launch ファイル

起動するもの:
  1. Gazebo Fortress（clinic.sdf）
  2. ros_gz_bridge（config/bridge.yaml）
  3. base_link -> lidar_link の静的TF
  4. RViz2（rviz:=false で起動しない）

使い方:
  ros2 launch clinic_delivery sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('clinic_delivery')
    model_path = os.path.join(pkg_share, 'models')
    world_file = os.path.join(pkg_share, 'worlds', 'clinic.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    # Gazebo が model://delivery_bot を見つけられるようにする
    resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=model_path + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))

    gui = LaunchConfiguration('gui')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')

    # Gazebo Fortress の起動（-r: すぐに再生開始）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': '-r -v 3 ' + world_file}.items(),
        condition=IfCondition(gui))

    # 画面なし（-s: 計算だけ）。動作が重いとき用
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': '-r -s -v 3 ' + world_file}.items(),
        condition=UnlessCondition(gui))

    # ブリッジ（yaml でまとめて指定）
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config,
                     'use_sim_time': use_sim_time}],
        output='screen')

    # LiDAR の取り付け位置（model.sdf の sensor pose と一致させる）
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.08', '--y', '0', '--z', '0.05',
                   '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        resource_path,
        gazebo,
        gazebo_headless,
        bridge,
        lidar_tf,
        rviz,
    ])
