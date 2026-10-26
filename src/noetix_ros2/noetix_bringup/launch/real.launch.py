from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from noetix_bringup.robot_resources import load_robot_resource
from noetix_bringup.gmr_launch import declare_gmr_arguments, gmr_launch_actions


def _launch_robot(context):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    resource = load_robot_resource(robot_name)
    # Resolve optional GMR before creating hardware actions; disabled mode has no dependency on it.
    gmr_actions = gmr_launch_actions(context, robot_name)

    control_frequency_text = LaunchConfiguration("control_frequency_hz").perform(context).strip()
    control_frequency_hz = (
        float(control_frequency_text)
        if control_frequency_text
        else 1000.0
    )

    description_file = resource.real_description

    if description_file.suffix == ".xacro":
        description_value = Command([
            FindExecutable(name="xacro"),
            " ",
            str(description_file),
            " robot_name:=", robot_name,
            " robot_root:=", str(resource.robot_root),
            " config_root:=", str(resource.robot_root),
        ])
        robot_description = ParameterValue(
            description_value, value_type=str)
    elif description_file.suffix == ".urdf":
        robot_description = description_file.read_text(
            encoding="utf-8")
    else:
        raise RuntimeError(
            f"Unsupported real robot description: "
            f"{description_file}"
        )

    print(
        f"[Noetix Real] robot={resource.name} dof={resource.dof} "
        f"control={control_frequency_hz:.1f}Hz"
    )

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{
                "robot_description": robot_description,
            }],
        ),
        Node(
            package="noetix_bridge",
            executable="noetix_bridge_node",
            name="noetix_bridge_node",
            output="screen",
            parameters=[{
                "robot_name": robot_name,
                "control_frequency_hz": control_frequency_hz,
                "rt_realtime": ParameterValue(
                    LaunchConfiguration("rt_realtime"), value_type=bool),
                "rt_lock_memory": ParameterValue(
                    LaunchConfiguration("rt_lock_memory"), value_type=bool),
                "rt_strict": ParameterValue(
                    LaunchConfiguration("rt_strict"), value_type=bool),
                "rt_priority": ParameterValue(
                    LaunchConfiguration("rt_priority"), value_type=int),
                "rt_cpu": ParameterValue(
                    LaunchConfiguration("rt_cpu"), value_type=int),
            }],
        ),
        Node(
            package="noetix_bridge",
            executable="noetix_pose_mode_node",
            name="noetix_pose_mode_node",
            output="screen",
            parameters=[{
                "robot_name": robot_name,
            }],
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "device_id":
                    LaunchConfiguration(
                        "joy_device_id"),
                "deadzone":
                    LaunchConfiguration(
                        "joy_deadzone"),
                "autorepeat_rate":
                    20.0,
            }],
            condition=IfCondition(
                LaunchConfiguration(
                    "use_gamepad")
            ),
        ),
    ]

    if resource.real_rviz is not None:
        actions.insert(
            1,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(resource.real_rviz), "--ros-args", "--log-level", "warn"],
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("use_rviz")
                ),
            ),
        )

    return actions + gmr_actions


def generate_launch_description():
    return LaunchDescription([
        # launch转发子进程输出时也保留ROS按日志级别设置的颜色。
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        DeclareLaunchArgument(
            "robot_name",
            default_value="n3",
            description=(
                "Robot directory name under "
                "share/noetix_robots/"
            ),
        ),
        DeclareLaunchArgument(
            "control_frequency_hz",
            default_value="",
            description=(
                "Low-level control rate in Hz. Empty selects 1000 Hz."
            ),
        ),
        DeclareLaunchArgument(
            "rt_realtime",
            default_value="true",
            description="Request SCHED_FIFO for the hardware control loop",
        ),
        DeclareLaunchArgument(
            "rt_lock_memory",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "rt_strict",
            default_value="false",
            description="Fail startup if RT scheduling or mlock setup fails",
        ),
        DeclareLaunchArgument(
            "rt_priority",
            default_value="80",
        ),
        DeclareLaunchArgument(
            "rt_cpu",
            default_value="-1",
            description="CPU core for the control thread; -1 leaves affinity unchanged",
        ),
        DeclareLaunchArgument(
            "use_gamepad",
            default_value="true",
            description="Start joystick driver; mode bindings are handled by noetix_policy",
        ),
        DeclareLaunchArgument(
            "joy_device_id",
            default_value="0",
        ),
        DeclareLaunchArgument(
            "joy_deadzone",
            default_value="0.001",
            description="Raw joystick deadzone; policy command deadzone is applied separately",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz when a config is available",
        ),
        *declare_gmr_arguments(),
        OpaqueFunction(function=_launch_robot),
    ])
