"""Localization (AMCL on your saved map) + Nav2 navigation for the amr robot in Isaac Sim.

  ros2 launch amr_nav nav.launch.py
  ros2 launch amr_nav nav.launch.py map:=~/mobile_robot/src/amr_nav/maps/world1.yaml \
      world:=~/mobile_robot/src/amr/urdf/world1.usd

Starts:
  * Isaac Sim with the robot inside the world (robot at the world origin = map origin)
  * scan_merger: lidar + 4 ultrasonics -> /scan_merged
  * Nav2 bringup: map_server + AMCL (localization) and planner, controller (Regulated Pure
    Pursuit), behaviors, bt_navigator, waypoint_follower, velocity_smoother
  * RViz: map, costmaps, AMCL particles, planned path; "2D Goal Pose" sends a goal
AMCL starts at (0, 0, 0). If the robot is somewhere else, use "2D Pose Estimate" in RViz.
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
    amr = get_package_share_directory('amr')
    nav = get_package_share_directory('amr_nav')
    bringup = get_package_share_directory('nav2_bringup')

    # scoped groups: arguments given to an included launch must not leak back into this file
    isaac = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr, 'launch', 'isaac.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'teleop': 'false',
            'rviz': 'false',
        }.items())], scoped=True)

    merger = Node(
        package='amr_nav', executable='scan_merger.py', name='scan_merger', output='screen',
        parameters=[{'use_sim_time': True}])

    nav2 = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items())], scoped=True)

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='~/mobile_robot/src/amr/urdf/world1.usd'),
        DeclareLaunchArgument('map', default_value=os.path.join(nav, 'maps', 'world1.yaml'),
                              description='map saved from SLAM (.yaml)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(nav, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav_delay', default_value='20.0',
                              description='seconds to wait for Isaac before starting Nav2'),
        isaac,
        merger,
        rviz,
        # Nav2 waits until Isaac publishes /clock and TF, otherwise AMCL/costmaps start
        # with "no transform" warnings while the warehouse is still loading
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[nav2]),
    ])
