"""Navigate while mapping: slam_toolbox + Nav2 at the same time (no saved map, no AMCL).

  ros2 launch amr_nav slam_nav.launch.py                       # maze world
  ros2 launch amr_nav slam_nav.launch.py world:=~/mobile_robot/src/amr/urdf/world1.usd

Starts:
  * Isaac Sim with the robot inside the world (Home = world origin)
  * scan_merger: lidar + 4 ultrasonics -> /scan_merged
  * slam_toolbox (mapping) -> live /map and map -> odom (replaces AMCL + map_server)
  * Nav2 navigation (planner, controller, behaviors, bt_navigator, ...) on that live map;
    the planner may plan through unknown (grey) space, so goals can be set beyond the
    explored area
  * RViz: click "2D Goal Pose" anywhere -> robot drives there, map grows on the way
  * every run starts with an EMPTY map; the map built during the run is what Nav2 plans on
  * teleop is off (Nav2 drives); teleop:=true opens it anyway for manual moves
  * map_outline: the TRUE walls of the world (orange) in RViz, display only - SLAM and Nav2
    never use it. Shows whether the robot / SLAM map is inside and aligned with the real world.
    Made once per world from the Isaac geometry:
      ~/isaacsim_5.1/python.sh ~/mobile_robot/src/amr/scripts/isaac_omap.py \
          --world ~/mobile_robot/src/amr/urdf/world_maze.usd \
          --out ~/mobile_robot/src/amr_nav/maps/world_maze_isaac
      (then colcon build --packages-select amr_nav)

Save the map at the end (second terminal):
  ros2 run nav2_map_server map_saver_cli -f ~/mobile_robot/src/amr_nav/maps/maze \
      --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _outline(context):
    """Map outline for RViz only: maps/<world name>_isaac.yaml, made from the Isaac world."""
    nav = get_package_share_directory('amr_nav')
    choice = LaunchConfiguration('outline').perform(context)
    if choice.lower() in ('false', 'off', 'none', ''):
        return []
    if choice == 'auto':
        world = os.path.basename(os.path.expanduser(LaunchConfiguration('world').perform(context)))
        choice = os.path.join(nav, 'maps', os.path.splitext(world)[0] + '_isaac.yaml')
    choice = os.path.expanduser(choice)
    if not os.path.isfile(choice):
        return [LogInfo(msg=f'[outline] {choice} not found - make it with scripts/isaac_omap.py '
                            '(see README in the launch file); running without outline')]
    return [Node(package='amr_nav', executable='map_outline.py', name='map_outline',
                 output='screen', parameters=[{'map_yaml': choice, 'use_sim_time': True}])]


def _nav2(context):
    """Nav2 on the live SLAM map, started after nav_delay seconds.

    Paths are resolved here, at launch start: the timer fires later, when this file's launch
    arguments may already be out of scope (e.g. when coverage.launch.py includes this file).
    """
    bringup = get_package_share_directory('nav2_bringup')
    params_file = os.path.expanduser(LaunchConfiguration('params_file').perform(context))
    delay = float(LaunchConfiguration('nav_delay').perform(context))
    # same Nav2 settings as nav.launch.py, but the map is still being built: the planner may
    # go through unknown cells and the global costmap is a 30 x 30 m window around the robot
    # (otherwise it is only as big as the explored area and far goals are "off the costmap")
    params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'allow_unknown': 'true',
            'global_costmap.global_costmap.ros__parameters.rolling_window': 'true',
        },
        convert_types=True)
    nav2 = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items())], scoped=True)
    return [TimerAction(period=delay, actions=[nav2])]


def generate_launch_description():
    amr = get_package_share_directory('amr')
    nav = get_package_share_directory('amr_nav')

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
        parameters=[os.path.join(nav, 'config', 'slam_params.yaml'),
                    {'use_sim_time': True, 'scan_topic': '/scan_merged'}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='~/mobile_robot/src/amr/urdf/world_maze.usd'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(nav, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('nav_delay', default_value='20.0',
                              description='seconds to wait for Isaac before starting Nav2'),
        DeclareLaunchArgument('outline', default_value='auto',
                              description='true wall outline in RViz only (not used by SLAM/Nav2): '
                                          'auto = maps/<world>_isaac.yaml, a .yaml path, or false'),
        isaac,
        merger,
        slam,
        OpaqueFunction(function=_outline),
        rviz,
        OpaqueFunction(function=_nav2),
    ])
