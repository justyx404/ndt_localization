from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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

    # Localization Node
    localization_node = Node(
        package="ndt_localization",
        executable="localization_node",
        name="localization_node",
        output="screen",
        parameters=[
            {"odom_frame_id": LaunchConfiguration("odom_frame_id")},
            {"base_frame_id": LaunchConfiguration("base_frame_id")},
            {"map_frame_id": LaunchConfiguration("map_frame_id")},
            {"localization.ndt_resolution": 1.0},
            {"localization.ndt_step_size": 0.1},
            {"localization.ndt_trans_epsilon": 0.01},
            {"localization.ndt_max_iter": 30},
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
            localization_node,
        ]
    )
