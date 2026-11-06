"""Show the amr robot (urdf/amr.urdf) in RViz.

  ros2 launch amr spawn.launch.py             # wheel joints held at 0
  ros2 launch amr spawn.launch.py gui:=true   # sliders to turn the wheels
  ros2 launch amr spawn.launch.py odom:=false # odom comes from Isaac Sim instead

TF tree: odom -> base_ground -> base_link -> wheels, caster, sensors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr')
    with open(os.path.join(pkg, 'urdf', 'amr.urdf'), 'r') as f:
        robot_description = f.read()

    gui = LaunchConfiguration('gui')
    odom = LaunchConfiguration('odom')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Show joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('odom', default_value='true',
                              description='Publish a static odom -> base_ground frame'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': robot_description}],
             condition=UnlessCondition(gui)),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),

        # odom frame for RViz: fixed at the robot's start pose. When Isaac Sim runs,
        # it publishes the moving odom -> base_ground instead (launch with odom:=false).
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='odom_to_base_ground',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_ground'],
             condition=IfCondition(odom)),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'spawn.rviz')]),
    ])
