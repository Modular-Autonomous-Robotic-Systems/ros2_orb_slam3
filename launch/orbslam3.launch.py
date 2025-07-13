import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration("log-level", default = logger_default)
    logger_arg = launch.actions.DeclareLaunchArgument(
            "log-level",
            default_value=[logger_default],
            description="Logging level")

    camera_topic_default = '/camera'
    camera_topic = launch.substitutions.LaunchConfiguration("camera-topic", default = camera_topic_default)
    camera_topic_arg = launch.actions.DeclareLaunchArgument(
            "camera-topic",
            default_value=[camera_topic_default],
            description = 'Camera Topic name to listen to for images')

    settings_file_path_default = '/ws/ros_ws/src/slam/orb_slam3/config/Monocular/sitl.yaml'
    settings_file_path = launch.substitutions.LaunchConfiguration("settings-file-path", default = settings_file_path_default)
    settings_file_path_arg = launch.actions.DeclareLaunchArgument(
            "settings-file-path",
            default_value=[settings_file_path_default],
            description = 'Path to ORBSLAM3 settings file')
    
    vocab_file_path_default = '/ws/ros_ws/src/slam/orb_slam3/Vocabulary/ORBvoc.txt.bin'
    vocab_file_path = launch.substitutions.LaunchConfiguration("vocab-file-path", default = vocab_file_path_default)
    vocab_file_path_arg = launch.actions.DeclareLaunchArgument(
            "vocab-file-path",
            default_value=[vocab_file_path_default],
            description = 'Path to ORBSLAM3 vocabulary file')
    delayed_launch = launch.actions.TimerAction(
        period=3.0,  # Seconds
        actions=[
            launch_ros.actions.Node(
                package='slam',
                executable='orbslam3_mono_node',
                name='orbslam3_mono_node',
                respawn=False,
                prefix=['gdb -ex run -ex bt --args'],
                arguments=['--ros-args', '--log-level', logger],
                parameters = [{
                    'settings_file_path': settings_file_path,
                    'camera_topic_name': camera_topic,
                    'vocab_file_path': vocab_file_path
                    }],
                output='screen'),
            launch_ros.actions.Node(
                package='controllers',
                executable='orbslam3_monocular_controller',
                output='screen',
                namespace='/',
                name='orbslam3_controller',
                respawn=False,
                arguments=['--ros-args', '--log-level', logger],
                parameters = [{
                    'settings_file_path': settings_file_path,
                    'camera_topic_name': camera_topic,
                    'vocab_file_path': vocab_file_path
                }]
            )
        ]
    )

    nodes = [
        logger_arg,
        camera_topic_arg,
        settings_file_path_arg,
        vocab_file_path_arg,
        delayed_launch
    ]

    return launch.LaunchDescription(nodes)
