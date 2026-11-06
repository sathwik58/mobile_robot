"""Build a map with slam_toolbox while you drive the amr robot in Isaac Sim.

  ros2 launch amr_nav slam.launch.py
  ros2 launch amr_nav slam.launch.py world:=~/mobile_robot/src/amr/urdf/world1.usd

Starts:
  * Isaac Sim with the robot inside the world (amr isaac.launch.py, its RViz turned off)
  * scan_merger: lidar + 4 ultrasonics fused into one 360 deg /scan_merged (base_ground)
  * slam_toolbox (online async) on /scan_merged -> /map and map -> odom
    (scan_topic:=/scan maps with the lidar only)
  * RViz with the growing map, lidar, ultrasonics and robot
  * teleop_twist_keyboard in a new terminal (drive slowly, see below)

When the map looks complete, save it from another terminal:
  ros2 run nav2_map_server map_saver_cli -f ~/mobile_robot/src/amr_nav/maps/world1 \
      --ros-args -p use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    amr = get_package_share_directory('amr')
    nav = get_package_share_directory('amr_nav')

    # GroupAction keeps rviz:=false (and the other arguments) inside the included launch;
    # without it they leak back here and switch off this file's RViz too
    isaac = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr, 'launch', 'isaac.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'teleop': LaunchConfiguration('teleop'),
            'rviz': 'false',
        }.items())], scoped=True)

    merger = Node(
        package='amr_nav', executable='scan_merger.py', name='scan_merger', output='screen',
        parameters=[{'use_sim_time': True}])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params'),
                    {'use_sim_time': True, 'scan_topic': LaunchConfiguration('scan_topic')}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='~/mobile_robot/src/amr/urdf/world1.usd',
                              description='Isaac world to map'),
        DeclareLaunchArgument('slam_params',
                              default_value=os.path.join(nav, 'config', 'slam_params.yaml')),
        DeclareLaunchArgument('scan_topic', default_value='/scan_merged',
                              description='/scan_merged (lidar + ultrasonics) or /scan (lidar only)'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        isaac,
        merger,
        slam,
        rviz,
    ])
