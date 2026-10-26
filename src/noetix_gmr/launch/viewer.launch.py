"""Display /noetix/gmt without opening a motion-capture SDK connection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # 日志颜色由ROS统一设置，不改变日志内容和输出级别。
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        DeclareLaunchArgument('robot', default_value='n3', choices=['n3', 'g1', 'e1', 'bumi3']),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('topic', default_value='/noetix/gmt'),
        DeclareLaunchArgument('width', default_value='960'),
        DeclareLaunchArgument('height', default_value='720'),
        Node(package='noetix_gmr', executable='mujoco_viewer_node',
             name='noetix_gmr_viewer', output='screen', parameters=[{
                 'robot': ParameterValue(LaunchConfiguration('robot'), value_type=str),
                 'model_path': ParameterValue(LaunchConfiguration('model_path'), value_type=str),
                 'topic': ParameterValue(LaunchConfiguration('topic'), value_type=str),
                 'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                 'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
             }]),
    ])
