"""Optional live retargeting shared by real and Gazebo launch entrances."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            ResetLaunchConfigurations)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

GMR_ROBOTS = {"bumi2": "bumi3", "bumi3": "bumi3", "bumi4340": "bumi3", "n3": "n3"}
# 与独立retarget启动入口保持相同参数，统一转发显式填写的值。
# Defaults are supplied by the independent retarget launch, only explicit overrides are forwarded.
GMR_ARGUMENTS = {
    'device': ("", 'Device: pico, xsens, optitrack, fzmotion'),
    'robot': ("", 'Optional GMR preset; must match the preset selected by robot_name'),
    'server_ip': ("", 'FZMotion or Motive host IP'),
    'client_ip': ("", 'Local IP for OptiTrack'),
    'xsens_port': ("", 'MVN UDP receive port'),
    'subject_filter': ("", 'FZMotion body name; empty accepts any tracked body'),
    'position_scale': ("", 'FZMotion position units to meters'),
    'model_path': ("", 'Override robot MJCF with an absolute path'),
    'ik_config': ("", 'Override IK JSON with an absolute path'),
    'human_height': ("", 'Operator height in meters; device-specific default'),
    'ik_damping': ("", 'IK damping; device-specific default'),
    'publish_hz': ("", 'Maximum ROS publication rate; only fresh samples are published'),
    'frame_timeout': ("", 'Pause publication after this many seconds without fresh IK data'),
    'warmup_iterations': ("", 'IK iterations on first valid source frame'),
    'offset_to_ground': ("", 'Enable ground alignment'),
    'pico_startup_timeout': ("", 'Wait for the first valid Pico body frame'),
    'normalize_initial_yaw': ("", 'Normalize initial Pico heading'),
    'interpolation_delay_ms': ("", 'Output interpolation delay'),
    'spine_joint': ("", 'Optional source spine bone override'),
    'spine_deg': ("", 'Spine pitch offset in degrees; zero disables it'),
    'pelvis_z_offset': ("", 'Publication pelvis height offset'),
    'lin_vel_alpha': ("", 'Linear velocity EMA coefficient'),
    'ang_vel_alpha': ("", 'Angular velocity EMA coefficient'),
    'lin_vel_max': ("", 'Linear velocity spike threshold; zero disables'),
    'ang_vel_max': ("", 'Angular velocity spike threshold; zero disables'),
    'motion_scale': ("", 'Joint motion amplitude multiplier'),
    'root_z_offset': ("", 'Root height adjustment before velocity calculation'),
    'root_xy_scale': ("", 'Root XY displacement multiplier'),
    'max_joint_speed': ("", 'Output joint speed limit; zero disables'),
    'lock_root_tilt': ("", 'Keep root yaw and remove roll/pitch'),
    'visualize': ("", 'Show the robot in a separate MuJoCo window'),
    'viewer_width': ("", 'MuJoCo viewer width in pixels'),
    'viewer_height': ("", 'MuJoCo viewer height in pixels'),
}


def declare_gmr_arguments():
    """Empty overrides let us distinguish omitted parameters from supplied values."""
    result = [
        DeclareLaunchArgument('gmr', default_value='auto', choices=['auto', 'true', 'false'],
                              description='auto: start GMR when any retargeting argument is supplied'),
        DeclareLaunchArgument('launch_gmr', default_value='auto', choices=['auto', 'true', 'false'],
                              description='Compatibility alias for gmr'),
    ]
    for name, (_, description) in GMR_ARGUMENTS.items():
        result.append(DeclareLaunchArgument(name, default_value='', description=description))
        result.append(DeclareLaunchArgument('gmr_' + name, default_value='',
                                           description='Compatibility alias for ' + name))
    return result


def _value(context, name):
    return LaunchConfiguration(name, default='').perform(context).strip()


def gmr_launch_actions(context, robot_name):
    flags = [_value(context, name).lower() for name in ('gmr', 'launch_gmr')]
    if any(flag not in ('', 'auto', 'true', 'false') for flag in flags):
        raise ValueError('gmr and launch_gmr must be auto, true or false')
    explicit = {flag for flag in flags if flag in ('true', 'false')}
    if len(explicit) > 1:
        raise ValueError('gmr and launch_gmr disagree; specify only one switch')
    if explicit == {'false'}:
        return []

    arguments = {}
    for name in GMR_ARGUMENTS:
        direct, alias = _value(context, name), _value(context, 'gmr_' + name)
        if direct and alias and direct != alias:
            raise ValueError(f'{name} and gmr_{name} disagree')
        if direct or alias:
            arguments[name] = direct or alias
    if not arguments and explicit != {'true'}:
        return []
    if robot_name not in GMR_ROBOTS:
        raise RuntimeError(f'No GMR source preset for robot_name={robot_name}')
    source_robot = GMR_ROBOTS[robot_name]
    if arguments.get('robot', source_robot) != source_robot:
        raise ValueError(f'robot_name={robot_name} requires GMR robot={source_robot}; '
                         'a different source would mismatch policy joints')
    arguments['robot'] = source_robot
    try:
        share = Path(get_package_share_directory('noetix_gmr'))
    except PackageNotFoundError as exc:
        raise RuntimeError('GMR was requested but noetix_gmr is unavailable. Build noetix_gmr '
                           'and source install/setup.bash.') from exc
    # Scope source arguments/environment and keep independent process exit behavior.
    return [GroupAction(actions=[ResetLaunchConfigurations(), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'retarget.launch.py')),
        launch_arguments=arguments.items(),
    )], scoped=True)]
