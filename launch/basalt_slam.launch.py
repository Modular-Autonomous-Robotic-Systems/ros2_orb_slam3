import launch
import launch_ros


def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration(
        "log-level", default=logger_default
    )
    logger_arg = launch.actions.DeclareLaunchArgument(
        "log-level", default_value=[logger_default], description="Logging level"
    )

    camera_topic_default = "/camera"
    camera_topic = launch.substitutions.LaunchConfiguration(
        "camera-topic", default=camera_topic_default
    )
    camera_topic_arg = launch.actions.DeclareLaunchArgument(
        "camera-topic",
        default_value=[camera_topic_default],
        description="Camera Topic name to listen to for images",
    )

    calibration_file_path_default = ""
    calibration_file_path = launch.substitutions.LaunchConfiguration(
        "calibration-file-path", default=calibration_file_path_default
    )
    calibration_file_path_arg = launch.actions.DeclareLaunchArgument(
        "calibration-file-path",
        default_value=[calibration_file_path_default],
        description="Path to Basalt Calibration File",
    )

    configuration_file_path_default = ""
    configuration_file_path = launch.substitutions.LaunchConfiguration(
        "configuration-file-path", default=configuration_file_path_default
    )
    configuration_file_path_arg = launch.actions.DeclareLaunchArgument(
        "configuration-file-path",
        default_value=[configuration_file_path_default],
        description="Path to Basalt Configuration File",
    )

    slam_type_default = "VSLAM"
    slam_type = launch.substitutions.LaunchConfiguration(
        "slam-type", default=slam_type_default
    )
    slam_type_arg = launch.actions.DeclareLaunchArgument(
        "slam-type",
        default_value=[slam_type_default],
        description="Type of SLAM to configure and activate. This node supports VSLAM and VISLAM",
    )

    basalt_slam_node = launch_ros.actions.Node(
        package="slam",
        executable="basalt_slam_node",
        name="basalt_slam_node",
        output="screen",
        namespace="/",
        respawn=False,
        arguments=["--ros-args", "--log-level", logger],
        parameters=[
            {
                "camera_topic_name": camera_topic,
                "calibration_file_path": calibration_file_path,
                "configuration_file_path": configuration_file_path,
                "slam_type": slam_type,
            }
        ],
    )

    nodes = [
        logger_arg,
        camera_topic_arg,
        calibration_file_path_arg,
        configuration_file_path_arg,
        slam_type_arg,
        basalt_slam_node,
    ]

    return launch.LaunchDescription(nodes)
