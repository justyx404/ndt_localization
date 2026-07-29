"""Replay one bag with strict separation between algorithm and reference topics."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    output_directory = LaunchConfiguration("output_directory")
    run_name = LaunchConfiguration("run_name")
    rate = LaunchConfiguration("rate")
    start_offset = LaunchConfiguration("start_offset")
    config_file = LaunchConfiguration("config_file")
    qos_overrides = LaunchConfiguration("qos_overrides")

    arguments = [
        DeclareLaunchArgument(
            "bag_path", description="Absolute path to the ROS 2 bag directory"
        ),
        DeclareLaunchArgument(
            "output_directory",
            description="Directory for CSV, JSON, and Markdown output",
        ),
        DeclareLaunchArgument("run_name", default_value="benchmark"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("start_offset", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_delay", default_value="10.0"),
        DeclareLaunchArgument("translation_x", default_value="0.0"),
        DeclareLaunchArgument("translation_y", default_value="0.0"),
        DeclareLaunchArgument("translation_z", default_value="0.0"),
        DeclareLaunchArgument("yaw_degrees", default_value="0.0"),
        DeclareLaunchArgument("position_sigma", default_value="0.25"),
        DeclareLaunchArgument("yaw_sigma_degrees", default_value="5.0"),
        DeclareLaunchArgument("deadline_ms", default_value="80.0"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ndt_localization"),
                    "config",
                    "localization.yaml",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "qos_overrides",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ndt_localization"),
                    "config",
                    "replay_qos.yaml",
                ]
            ),
        ),
    ]

    localizer = Node(
        package="ndt_localization",
        executable="localization_node",
        name="localization_node",
        output="screen",
        parameters=[config_file, {"use_sim_time": True}],
        remappings=[
            ("/initialpose", "/benchmark/initialpose"),
            ("/odometry_map", "/benchmark/odometry_map"),
            ("/localization/scan_diagnostics", "/benchmark/scan_diagnostics"),
            ("/tf", "/benchmark/tf"),
        ],
    )

    evaluator = Node(
        package="ndt_localization",
        executable="localization_benchmark_evaluator.py",
        name="localization_benchmark_evaluator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "output_directory": output_directory,
                "run_name": run_name,
                "bag_path": bag_path,
                "replay_rate": rate,
                "start_offset": start_offset,
                "deadline_ms": LaunchConfiguration("deadline_ms"),
                "config_file": config_file,
                "initial_pose_delay": LaunchConfiguration("initial_pose_delay"),
                "translation_x": LaunchConfiguration("translation_x"),
                "translation_y": LaunchConfiguration("translation_y"),
                "translation_z": LaunchConfiguration("translation_z"),
                "yaw_degrees": LaunchConfiguration("yaw_degrees"),
                "position_sigma": LaunchConfiguration("position_sigma"),
                "yaw_sigma_degrees": LaunchConfiguration(
                    "yaw_sigma_degrees"
                ),
            }
        ],
    )

    initial_pose = Node(
        package="ndt_localization",
        executable="synthetic_initial_pose.py",
        name="synthetic_initial_pose",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "delay_seconds": LaunchConfiguration("initial_pose_delay"),
                "translation_x": LaunchConfiguration("translation_x"),
                "translation_y": LaunchConfiguration("translation_y"),
                "translation_z": LaunchConfiguration("translation_z"),
                "yaw_degrees": LaunchConfiguration("yaw_degrees"),
                "position_sigma": LaunchConfiguration("position_sigma"),
                "yaw_sigma_degrees": LaunchConfiguration("yaw_sigma_degrees"),
            }
        ],
    )

    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "100",
            "--rate",
            rate,
            "--start-offset",
            start_offset,
            "--disable-keyboard-controls",
            "--qos-profile-overrides-path",
            qos_overrides,
            "--topics",
            "/global_map",
            "/odometry_lio",
            "/cloud_registered_body",
            "/tf",
            "/odometry_map",
            "/initialpose",
            "--remap",
            "/tf:=/reference/tf",
            "/odometry_map:=/reference/odometry_map",
            "/initialpose:=/reference/initialpose",
        ],
        output="screen",
    )
    delayed_bag_player = TimerAction(period=2.0, actions=[bag_player])
    shutdown_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[
                LogInfo(msg="Bag replay finished; finalizing benchmark artifacts"),
                EmitEvent(event=Shutdown(reason="bag replay completed")),
            ],
        )
    )

    return LaunchDescription(
        arguments
        + [
            localizer,
            evaluator,
            initial_pose,
            delayed_bag_player,
            shutdown_when_complete,
        ]
    )
