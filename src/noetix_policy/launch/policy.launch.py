"""启动 C++ 原生策略节点，与独立 Python 工程使用相同的包名和参数接口。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """保留手柄配置、仿真时钟和屏幕日志接口，启动原生C++切换器。"""
    config = PathJoinSubstitution([FindPackageShare("noetix_policy"), "config/joystick.yaml"])
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        DeclareLaunchArgument("joystick_config", default_value=config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="noetix_policy", executable="policy_node", output="screen", parameters=[{
            "joystick_config": LaunchConfiguration("joystick_config"),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }]),
    ])
