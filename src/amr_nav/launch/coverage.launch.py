"""Part C - corridor lane coverage (boustrophedon) in Isaac Sim.

  ros2 launch amr_nav coverage.launch.py
  ros2 launch amr_nav coverage.launch.py world:=~/mobile_robot/src/amr/urdf/<corridor>.usd

Starts everything from slam_nav.launch.py (Isaac with the corridor world, scan_merger,
slam_toolbox, Nav2, true-wall outline) plus lane_coverage.py, which
  1. measures the corridor width / direction / length with the lidar,
  2. computes the number of lanes and their offsets,
  3. drives the lanes back and forth (lidar lane keeping, ultrasonic + lidar lane ends),
  4. returns Home with Nav2 and writes a coverage report (~/mobile_robot/coverage_reports).
New corridor:  python3 ~/mobile_robot/src/amr/scripts/make_corridor.py --width 2.8 --length 10
Settings:      config/coverage_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav = get_package_share_directory('amr_nav')

    base = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'slam_nav.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'outline': LaunchConfiguration('outline'),
            'nav_delay': LaunchConfiguration('nav_delay'),
            'rviz': 'false',
        }.items())], scoped=True)

    coverage = Node(
        package='amr_nav', executable='lane_coverage.py', name='lane_coverage', output='screen',
        parameters=[LaunchConfiguration('params'), {'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav, 'rviz', 'coverage.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='~/mobile_robot/src/amr/urdf/world_corridor.usd'),
        DeclareLaunchArgument('params', default_value=os.path.join(nav, 'config', 'coverage_params.yaml')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('outline', default_value='auto'),
        DeclareLaunchArgument('nav_delay', default_value='20.0'),
        DeclareLaunchArgument('start_after', default_value='30.0',
                              description='seconds before the coverage node starts (Isaac + Nav2 up)'),
        base,
        rviz,
        TimerAction(period=LaunchConfiguration('start_after'), actions=[coverage]),
    ])
