import launch_ros

import launch


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

    imu_topic_default = ""
    imu_topic = launch.substitutions.LaunchConfiguration(
        "imu-topic", default=imu_topic_default
    )
    imu_topic_arg = launch.actions.DeclareLaunchArgument(
        "imu-topic",
        default_value=[imu_topic_default],
        description="IMU Topic name (required for VISLAM)",
    )

    use_sim_time_default = "False"
    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        "use-sim-time",
        default_value=[use_sim_time_default],
        choices=["True", "False"],
        description="Boolean, the only accepted values are True and False. Run the SLAM compute node on the simulator clock published on /clock, so that the stamps it produces on ~/annotated_frame and on its pose output share a domain with the stamps it consumes.",
    )
    use_sim_time = launch.substitutions.LaunchConfiguration(
        "use-sim-time", default=use_sim_time_default
    )

    use_visualisation_default = "False"
    use_visualisation_arg = launch.actions.DeclareLaunchArgument(
        "use-visualisation",
        default_value=[use_visualisation_default],
        choices=["True", "False"],
        description="Boolean, the only accepted values are True and False. Open the Basalt Pangolin window and render the live VIO trajectory, image grid, sliding-window landmarks and local map alongside tracking. Requires a reachable X display, so leave it False for headless runs.",
    )
    use_visualisation = launch.substitutions.LaunchConfiguration(
        "use-visualisation", default=use_visualisation_default
    )

    basalt_slam_node = launch_ros.actions.Node(
        package="slam",
        executable="basalt_slam_node",
        name="basalt_slam_node",
        output="screen",
        namespace="/",
        respawn=False,
        arguments=["--ros-args", "--log-level", logger],
        prefix='gdb -ex "set confirm off" -ex run -ex bt -ex q --args',
        parameters=[
            {
                "camera_topic_name": camera_topic,
                "calibration_file_path": calibration_file_path,
                "configuration_file_path": configuration_file_path,
                "slam_type": slam_type,
                "imu_topic_name": imu_topic,
                "use_sim_time": use_sim_time,
                "use_visualisation": use_visualisation,
            }
        ],
    )

    nodes = [
        logger_arg,
        camera_topic_arg,
        calibration_file_path_arg,
        configuration_file_path_arg,
        slam_type_arg,
        imu_topic_arg,
        use_sim_time_arg,
        use_visualisation_arg,
        basalt_slam_node,
    ]

    return launch.LaunchDescription(nodes)
