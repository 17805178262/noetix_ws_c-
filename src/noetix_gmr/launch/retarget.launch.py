"""Launch one device/robot retargeting pipeline on /noetix/gmt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Empty optional values preserve device-specific defaults in the node.
ARGUMENTS = {
    'device': ('pico', str, 'Device: pico, xsens, optitrack, fzmotion'),
    'robot': ('n3', str, 'Robot: n3, g1, e1, bumi3'),
    'server_ip': ('', str, 'FZMotion or Motive host IP'),
    'client_ip': ('', str, 'Local IP for OptiTrack'),
    'xsens_port': ('9763', int, 'MVN UDP receive port'),
    'subject_filter': ('', str, 'FZMotion body name; empty accepts any tracked body'),
    'position_scale': ('0.001', float, 'FZMotion position units to meters'),
    'model_path': ('', str, 'Override robot MJCF with an absolute path'),
    'ik_config': ('', str, 'Override IK JSON with an absolute path'),
    'human_height': ('', float, 'Operator height in meters; device-specific default'),
    'ik_damping': ('', float, 'IK damping; device-specific default'),
    'publish_hz': ('50.0', float, 'Maximum ROS publication rate; only fresh samples are published'),
    'frame_timeout': ('0.5', float, 'Pause publication after this many seconds without fresh IK data'),
    'warmup_iterations': ('1000', int, 'IK iterations on first valid source frame'),
    'offset_to_ground': ('true', bool, 'Enable ground alignment'),
    'pico_startup_timeout': ('30.0', float, 'Wait for the first valid Pico body frame'),
    'normalize_initial_yaw': ('false', bool, 'Normalize initial Pico heading'),
    'interpolation_delay_ms': ('0.0', float, 'Output interpolation delay'),
    'spine_joint': ('', str, 'Optional source spine bone override'),
    'spine_deg': ('', float, 'Spine pitch offset in degrees; zero disables it'),
    'pelvis_z_offset': ('', float, 'Publication pelvis height offset'),
    'lin_vel_alpha': ('1.0', float, 'Linear velocity EMA coefficient'),
    'ang_vel_alpha': ('1.0', float, 'Angular velocity EMA coefficient'),
    'lin_vel_max': ('0.0', float, 'Linear velocity spike threshold; zero disables'),
    'ang_vel_max': ('0.0', float, 'Angular velocity spike threshold; zero disables'),
    'motion_scale': ('1.0', float, 'Joint motion amplitude multiplier'),
    'root_z_offset': ('0.0', float, 'Root height adjustment before velocity calculation'),
    'root_xy_scale': ('1.0', float, 'Root XY displacement multiplier'),
    'max_joint_speed': ('0.0', float, 'Output joint speed limit; zero disables'),
    'lock_root_tilt': ('false', bool, 'Keep root yaw and remove roll/pitch'),
}


def launch_node(context):
    parameters = {}
    for name, (_, value_type, _) in ARGUMENTS.items():
        value = LaunchConfiguration(name).perform(context)
        if value == '':
            continue
        if value_type is bool:
            if value.lower() not in ('true', 'false'):
                raise ValueError(f'{name} must be true or false')
            parameters[name] = ParameterValue(value.lower() == 'true', value_type=bool)
        else:
            parameters[name] = ParameterValue(value_type(value), value_type=value_type)
    nodes = [Node(package='noetix_gmr', executable='retarget_node',
                  name='noetix_gmr', output='screen', parameters=[parameters])]
    if LaunchConfiguration('visualize').perform(context) == 'true':
        viewer_parameters = {
            'robot': parameters['robot'],
            'width': ParameterValue(LaunchConfiguration('viewer_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('viewer_height'), value_type=int),
        }
        if 'model_path' in parameters:
            viewer_parameters['model_path'] = parameters['model_path']
        nodes.append(Node(package='noetix_gmr', executable='mujoco_viewer_node',
                          name='noetix_gmr_viewer', output='screen', parameters=[viewer_parameters]))
    return nodes


def generate_launch_description():
    arguments = [DeclareLaunchArgument(name, default_value=default, description=description)
                 for name, (default, _, description) in ARGUMENTS.items()]
    arguments += [
        DeclareLaunchArgument('visualize', default_value='false', choices=['true', 'false'],
                              description='Show the robot in a MuJoCo window'),
        DeclareLaunchArgument('viewer_width', default_value='960'),
        DeclareLaunchArgument('viewer_height', default_value='720'),
    ]
    # 独立启动和联合启动都使用ROS原生终端配色。
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        *arguments,
        OpaqueFunction(function=launch_node),
    ])
