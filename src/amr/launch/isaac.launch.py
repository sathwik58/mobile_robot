"""Isaac Sim + RViz + keyboard teleop for the amr robot.

  ros2 launch amr isaac.launch.py
  ros2 launch amr isaac.launch.py teleop:=false rviz:=false
  ros2 launch amr isaac.launch.py world:=~/mobile_robot/src/amr/urdf/world1.usd
  ros2 launch amr isaac.launch.py usd:=/path/to/other.usd isaac_dir:=~/isaacsim_5.1

Starts:
  * Isaac Sim 5.1 with urdf/amr.usd (robot + sensors + ROS 2 graph), ground plane, Play
  * robot_state_publisher (URDF + /joint_states from Isaac) with sim time
  * RViz (robot, TF, lidar, ultrasonics, odometry, camera images)
  * teleop_twist_keyboard in its own terminal window -> /cmd_vel

Isaac Sim publishes odom -> base_ground, so no static odom frame is started here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _isaac(context):
    isaac_dir = os.path.expanduser(LaunchConfiguration('isaac_dir').perform(context))
    usd = os.path.expanduser(LaunchConfiguration('usd').perform(context))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    world = os.path.expanduser(LaunchConfiguration('world').perform(context))
    script = os.path.join(get_package_share_directory('amr'), 'scripts', 'isaac_run.py')

    # Isaac Sim runs Python 3.11 with its own internal ROS 2 Humble libraries; the
    # system ROS (Python 3.10) must not leak into its environment.
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    ros_libs = [p for p in env.get('LD_LIBRARY_PATH', '').split(':') if p and '/opt/ros' not in p]
    bridge_lib = os.path.join(isaac_dir, 'exts', 'isaacsim.ros2.bridge', 'humble', 'lib')
    env['LD_LIBRARY_PATH'] = ':'.join([bridge_lib] + ros_libs)
    env['ROS_DISTRO'] = 'humble'
    env.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    cmd = [os.path.join(isaac_dir, 'python.sh'), script, '--usd', usd]
    if world:
        cmd += ['--world', world]
    if headless:
        cmd.append('--headless')
    return [ExecuteProcess(cmd=cmd, env=env, output='screen', name='isaac_sim')]


def generate_launch_description():
    pkg = get_package_share_directory('amr')
    with open(os.path.join(pkg, 'urdf', 'amr.urdf'), 'r') as f:
        robot_description = f.read()

    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('isaac_dir', default_value='~/isaacsim_5.1',
                              description='Isaac Sim 5.1 install folder'),
        DeclareLaunchArgument('usd', default_value='~/mobile_robot/src/amr/urdf/amr.usd',
                              description='Stage with robot, sensors and ROS 2 graph'),
        DeclareLaunchArgument('world', default_value='',
                              description='World .usd loaded around the robot (empty = ground only)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Isaac Sim without a window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='Open teleop_twist_keyboard in a new terminal'),

        OpaqueFunction(function=_isaac),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'isaac.rviz')],
             parameters=[sim_time],
             condition=IfCondition(LaunchConfiguration('rviz'))),

        Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
             prefix='gnome-terminal --wait --', output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
