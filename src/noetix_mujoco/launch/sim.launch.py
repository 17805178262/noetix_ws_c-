#!/usr/bin/env python3
"""Independent MuJoCo backend, same Noetix ROS 2 interface as Gazebo/real."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from noetix_bringup.gmr_launch import declare_gmr_arguments, gmr_launch_actions


def _launch_robot(context):
    robot = LaunchConfiguration('robot_name').perform(context).strip()
    robots_share = Path(get_package_share_directory('noetix_robots'))
    robot_root = robots_share / robot
    if robot not in ('n3','bumi2','bumi3','bumi4340') or not (robot_root/'config/robot.yaml').is_file():
        raise ValueError('Unknown robot_name: '+robot)
    cfg_dir = Path(get_package_share_directory('noetix_mujoco')) / 'config'
    urdf = __import__('yaml').safe_load((robot_root/'config/robot.yaml').read_text())['robot']['description']['real']
    urdf_path = robot_root / urdf
    robot_description = urdf_path.read_text()
    sim_override = LaunchConfiguration('sim_config').perform(context).strip()
    gui = LaunchConfiguration('gui').perform(context).strip().lower()
    headless = gui in ('false','0','no','off')

    backend = Node(
        package='noetix_mujoco', executable='mujoco_backend', name='mujoco_backend',
        output='screen',
        arguments=['--robot-root',str(robot_root),'--config-dir',str(cfg_dir),
                   '--sim-config',sim_override] + (['--headless'] if headless else []),
    )
    state_publisher = Node(
        package='robot_state_publisher',executable='robot_state_publisher',
        name='robot_state_publisher',output='screen',parameters=[{
            'robot_description':robot_description,
            'use_sim_time':True,
        }],
    )
    pose_mode = Node(
        package='noetix_bridge',executable='noetix_pose_mode_node',
        name='noetix_pose_mode_node',output='screen',parameters=[{
            'robot_name':robot,
            'use_sim_time':True,
            'trajectory_use_ros_time':True,
        }],
    )
    joy = Node(
        package='joy',executable='joy_node',name='joy_node',output='screen',
        condition=IfCondition(LaunchConfiguration('use_gamepad')),parameters=[{
            'device_id':ParameterValue(LaunchConfiguration('joy_device_id'),value_type=int),
            'deadzone':ParameterValue(LaunchConfiguration('joy_deadzone'),value_type=float),
            'autorepeat_rate':20.0,
            'use_sim_time':True,
        }],
    )
    actions = [backend,state_publisher,pose_mode,joy]
    if LaunchConfiguration('launch_policy').perform(context).strip().lower() in ('true','1','yes','on'):
        policy_share = Path(get_package_share_directory('noetix_policy'))
        joystick_override = LaunchConfiguration('joystick_config').perform(context).strip()
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(policy_share/'launch/policy.launch.py')),
            launch_arguments={
                'use_sim_time':'true',
                'joystick_config': joystick_override or str(policy_share/'config/joystick.yaml'),
            }.items(),
        ))
    return actions + gmr_launch_actions(context,robot)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',default_value='n3',
                              description='n3, bumi2, bumi3, bumi4340'),
        DeclareLaunchArgument('gui',default_value='true',description='MuJoCo viewer on/off'),
        DeclareLaunchArgument('use_gamepad',default_value='true'),
        DeclareLaunchArgument('joy_device_id',default_value='0'),
        DeclareLaunchArgument('joy_deadzone',default_value='0.001'),
        DeclareLaunchArgument('sim_config',default_value='',
                              description='Optional MuJoCo simulation.yaml override'),
        DeclareLaunchArgument('launch_policy',default_value='false'),
        DeclareLaunchArgument('joystick_config',default_value=''),
        *declare_gmr_arguments(),
        OpaqueFunction(function=_launch_robot),
    ])
