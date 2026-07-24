from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    odom_frame_id_arg = DeclareLaunchArgument(
        "odom_frame_id", default_value="camera_init", description="Odometry frame ID"
    )

    base_frame_id_arg = DeclareLaunchArgument(
        "base_frame_id", default_value="base_link", description="Base frame ID"
    )

    map_frame_id_arg = DeclareLaunchArgument(
        "map_frame_id", default_value="map", description="Map frame ID"
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ndt_localization"), "config", "localization.yaml"]
        ),
        description="Localization parameter file",
    )

    # Localization Node
    localization_node = Node(
        package="ndt_localization",
        executable="localization_node",
        name="localization_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"odom_frame_id": LaunchConfiguration("odom_frame_id")},
            {"base_frame_id": LaunchConfiguration("base_frame_id")},
            {"map_frame_id": LaunchConfiguration("map_frame_id")},
        ],
        remappings=[
            ("/global_map", "/global_map"),
            ("/odometry_lio", "/odometry_lio"),
            ("/cloud_registered_body", "/cloud_registered_body"),
            ("/initialpose", "/initialpose"),
            ("/odometry_map", "/odometry_map"),
        ],
    )

    return LaunchDescription(
        [
            odom_frame_id_arg,
            base_frame_id_arg,
            map_frame_id_arg,
            config_file_arg,
            localization_node,
        ]
    )
