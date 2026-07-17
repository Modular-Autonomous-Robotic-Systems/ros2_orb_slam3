from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    "/ws/ros_ws/src/tello/tello_driver/launch/sim.launch.py"
                )
            ),
            ExecuteProcess(
                cmd=[
                    "/ws/ros_ws/build/slam/basalt_slam_node",
                    "--ros-args",
                    "-p",
                    "camera_topic_name:=/drone1/image_raw",
                    "-p",
                    "calibration_file_path:=/ws/ros_ws/src/slam/ext/basalt/data/tello_calib.json",
                    "-p",
                    "configuration_file_path:=/ws/ros_ws/src/slam/ext/basalt/data/euroc_config.json",
                ],
                output="screen",
            ),
        ]
    )

