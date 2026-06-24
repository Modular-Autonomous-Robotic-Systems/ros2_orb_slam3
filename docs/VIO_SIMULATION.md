# VIO Simulation Setup

## 1. Introduction

### 1.1 Purpose of This Document

This document describes the design, implementation, and current status of the Visual Inertial Odometry (VIO) simulation setup for the autonomous Tello drone project. It is intended to serve as both a technical reference and a progress tracker for the integration of the Basalt SLAM algorithm with the Tello Gazebo simulation environment. Anyone picking up this work should be able to read this document and understand what has been built, how it fits together, what is working, and what remains to be done.

### 1.2 Background

Autonomous drone navigation requires the drone to know its position and orientation in the world at all times. One of the most effective ways to achieve this without GPS — particularly in indoor environments — is Visual Inertial Odometry. VIO uses a camera and an IMU to continuously estimate the drone's 6-DOF pose (position and rotation) by tracking visual features across frames and fusing that information with inertial measurements.

The Basalt VIO library, developed at TU Munich, implements this algorithm using non-linear factor graph optimization. It is capable of real-time pose estimation and map building from monocular or stereo camera input. This project integrates Basalt into a ROS2-based Tello drone simulation to establish the foundational SLAM layer of the autonomous navigation stack.

### 1.3 Scope and Goals

The immediate goal of this integration is to establish a working data pipeline from the Gazebo simulation camera through to the Basalt SLAM node, verify that images are being received and processed, and lay the groundwork for full VIO once calibration files are available. The broader goal, as part of the research paper, is to demonstrate autonomous drone navigation using a SLAM-based map combined with a planning policy layer.

---

## 2. Launch Architecture

### 2.1 System Overview

The system connects the Gazebo simulation environment to the Basalt VIO algorithm through a ROS2 publish-subscribe pipeline. The Tello drone's simulated camera publishes image frames as ROS2 messages. The Basalt SLAM node subscribes to those frames, processes them through its visual odometry algorithm, and outputs a pose estimate. That pose is then available to higher-level planning systems.

### 2.2 Data Flow

```
[Gazebo Simulation]
       |
       |  Simulated camera rendered via libgazebo_ros_camera.so
       |  Physics and world managed by Gazebo engine
       v
[/drone1/image_raw]
       |
       |  ROS2 topic, type: sensor_msgs/Image
       |  Published at approximately 25 Hz
       v
[BasaltSLAMNode — node.cpp]
       |
       |  ROS2 lifecycle node activates and creates image subscriber
       |  Subscriber fires GrabImage() on each incoming frame
       |  Converts sensor_msgs::Image to cv::Mat using cv_bridge
       |  Extracts timestamp from image header
       |  Creates a Frame object containing {image, timestamp}
       v
[GrabImage()]
       |
       |  Entry point into the SLAM processing pipeline
       |  Hands the Frame to the slam.cpp wrapper layer
       v
[slam.cpp — Basalt Wrapper]
       |
       |  Converts cv::Mat to Basalt's internal image format
       |  Packages image and timestamp into an OpticalFlowInput structure
       |  Calls mpController->TrackMonocular()
       v
[TrackMonocular()]
       |
       |  Basalt core VIO algorithm
       |  Requires calibration_file_path and configuration_file_path to run
       |  Detects and tracks visual features across frames
       |  Estimates camera motion from feature displacement
       |  Outputs camera pose as an SE3 transformation (tcw)
       v
[Camera Pose]
       |
       |  Inverted from tcw to Twc (world-frame pose)
       |  Published as TF transform and on pose topic
       v
[Planning / Policy Layer]
       |
       |  Future work: exploration policy, path planning
       |  Feeds into the autonomous navigation paper contribution
```

### 2.3 Key Source Files

**`driver.cpp`**

The entry point of the Basalt SLAM node. It initializes ROS2, instantiates the `BasaltSLAMNode` object, and spins the node to keep it alive and processing callbacks. On a real drone it also manages UDP socket communication with the Tello SDK, handles command sending and telemetry reception, and publishes video frames from the drone to ROS2 topics. In simulation, the Gazebo plugin handles the camera, but `driver.cpp` still creates and drives the node. This file compiles into the executable `basalt_slam_node`.

**`node.cpp`**

The ROS2 lifecycle interface layer for the Basalt SLAM system. It is responsible for declaring and reading the three startup parameters, activating the lifecycle node, creating the image subscriber, receiving incoming `sensor_msgs::Image` messages, converting them to OpenCV format using `cv_bridge`, creating timestamped Frame objects, passing those frames to `slam.cpp`, and publishing processed output frames and pose transforms back to ROS2 topics. This file contains no VIO mathematics — it is purely the plumbing between ROS2 and the algorithm.

**`slam.cpp`**

The wrapper layer that sits between `node.cpp` and the Basalt library. When initialized, it reads the calibration and configuration files and creates the Basalt `Controller` object. When a frame arrives from `node.cpp`, it converts the OpenCV image into Basalt's internal image format, wraps it in an `OpticalFlowInput` structure with the associated timestamp, and calls `TrackMonocular()` on the Controller. The resulting SE3 pose is returned to `node.cpp` for publication. Note that `TrackMonocular()` will crash if called with empty or invalid calibration and configuration file paths.

---

## 3. Implementation

### 3.1 Launch File

The top-level launch file `vio_sim.launch.py` starts both the Tello Gazebo simulation and the Basalt SLAM node in a single command. It includes the existing simulation launch file and passes the required camera topic parameter directly to the Basalt node executable.

**File location:** `slam/launch/vio_sim.launch.py`

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([

        # Start the Tello Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                "/ws/ros_ws/src/tello/tello_driver/launch/sim.launch.py"
            )
        ),

        # Start the Basalt SLAM node and pass the camera topic parameter
        # TODO: add calibration_file_path and configuration_file_path
        ExecuteProcess(
            cmd=[
                "/ws/ros_ws/build/slam/basalt_slam_node",
                "--ros-args",
                "-p", "camera_topic_name:=/drone1/image_raw",
            ],
            output="screen"
        )

    ])
```

To launch the full pipeline:

```bash
ros2 launch slam vio_sim.launch.py
```

### 3.2 Parameters

The Basalt SLAM node reads three parameters at startup. These are declared inside `node.cpp` and must be supplied either through the launch file or as command-line overrides. All three must be present and valid for the node to progress past `TrackMonocular()`.

**`camera_topic_name`**

Specifies which ROS2 topic the node subscribes to for incoming camera frames. This must match the topic published by the Tello simulation. In the current setup this is set to `/drone1/image_raw`. The node will not receive any images if this topic name does not exactly match what the simulation publishes.

Current value: `/drone1/image_raw` — confirmed working.

**`calibration_file_path`**

Path to the camera calibration JSON file. This file contains the physical properties of the camera lens and sensor: focal length (fx, fy), optical center (cx, cy), and distortion coefficients. Basalt uses these values to convert 2D pixel coordinates into 3D rays, which is the geometric foundation of all pose estimation. Without a valid path, `TrackMonocular()` crashes immediately on the first received frame. Incorrect calibration leads to poor feature tracking, inaccurate pose estimates, or complete SLAM failure.

Current value: empty — causes crash.

**`configuration_file_path`**

Path to the Basalt algorithm configuration JSON file. This file contains tuning parameters for the VIO algorithm including feature detector type and thresholds, number of features to track per frame, keyframe selection policy, optimization window size, and IMU noise parameters. Without a valid path, `TrackMonocular()` crashes immediately on the first received frame.

Current value: empty — causes crash.

### 3.3 GrabImage() — Image Reception Callback

`GrabImage()` is the most important function in `node.cpp`. It is registered as the callback for the image subscriber and fires automatically every time a new frame arrives on `/drone1/image_raw`. This callback has been confirmed working — it is receiving frames from the simulator at approximately 25 Hz. The function converts the incoming ROS message to OpenCV format, extracts the timestamp, constructs a Frame object, and passes it down to `slam.cpp`. The crash currently occurs inside `TrackMonocular()` after this hand-off, not inside `GrabImage()` itself.

The following log line can be used to confirm callback execution during testing:

```cpp
RCLCPP_INFO(this->get_logger(), "[Basalt] IMAGE RECEIVED, stamp: %ld",
             msg->header.stamp.nanosec);
```

### 3.4 Dependency Configuration

Two environment variables must be set before launching the simulation. These are not code changes but environment configuration required to locate shared libraries at runtime.

`GAZEBO_PLUGIN_PATH` must include the directory containing `libTelloPlugin.so`. Without this, Gazebo cannot load the Tello drone plugin and the drone will not appear in the simulation.

`LD_LIBRARY_PATH` must include the tello_msgs install path containing `libtello_msgs__rosidl_typesupport_cpp.so`. Without this, the Tello driver node fails to start due to an unresolved shared library.

---

## 4. Validation

### 4.1 Completed Verification

The following items have been confirmed working through direct testing.

**Basalt node build**

All three source files compile without errors into a single executable:

```bash
./build/slam/basalt_slam_node
```

Output:
```
Initialising Slam Node
```

**Simulation launch and drone spawn**

The Gazebo simulation opens, the Tello drone model is inserted into the world, and the Tello plugin loads and attaches to the drone's physics body:

```
TELLO PLUGIN
-----------------------------------------
link_name: base_link_1
Successfully sent factory message to Gazebo
```

**Camera topic publishing**

Gazebo publishes `/drone1/image_raw` successfully. The topic appears in `ros2 topic list` and streams at approximately 25–28 Hz:

```bash
ros2 topic hz /drone1/image_raw
```

```
average rate: 26.XXX Hz
```

**Basalt lifecycle node activation**

The Basalt SLAM node activates through the ROS2 lifecycle successfully. The image subscriber is created during the activation step and begins listening on `/drone1/image_raw`.

**GrabImage() callback confirmed**

The `GrabImage()` callback is being called continuously as frames arrive from the simulator. Image data from the Gazebo simulation is successfully reaching the Basalt node. The pipeline from simulation camera to SLAM node entry point is fully operational.

### 4.2 Pending Verification

The following steps are blocked by the missing calibration and configuration files. They will be executed once those files are provided.

**TrackMonocular() execution**

```bash
# After adding calibration and config paths to launch file, relaunch:
ros2 launch slam vio_sim.launch.py
```

Expected: no crash on first frame. Basalt processes image and returns pose.

**Pose output**

```bash
ros2 topic echo /basalt/pose
```

Expected: continuous pose messages at frame rate.

**Subscription confirmation**

```bash
ros2 topic info /drone1/image_raw
```

Expected:
```
Publisher count: 1
Subscription count: 1
```

---

## 5. Current Blockers

### 5.1 Calibration File Not Provided — Basalt Crashes in TrackMonocular()

This is the active blocker. The Basalt node launches, activates, subscribes to the image topic, and `GrabImage()` receives frames correctly. However, when the first frame is passed into `TrackMonocular()`, Basalt crashes because `calibration_file_path` is empty. Basalt requires the calibration file to set up its internal camera model before it can process any images. With an empty path, the Controller has no camera geometry and fails immediately on the first call.

The calibration file (`calib.json`) must contain:
- Focal length: `fx`, `fy` — how the lens maps 3D rays to 2D pixels
- Optical center: `cx`, `cy` — the principal point of the image
- Distortion coefficients — how the lens warps straight lines
- Camera-IMU transform — spatial relationship between camera and IMU frames
- Image resolution — width and height in pixels


### 5.2 Configuration File Not Provided — Required by TrackMonocular()

The Basalt algorithm configuration file (`config.json`) is also empty. This file is read alongside the calibration file during `TrackMonocular()` initialization and sets the runtime behavior of the VIO algorithm — feature detector parameters, keyframe thresholds, optimization window size, and IMU noise model. Without it, the Controller cannot be configured and crashes alongside the missing calibration.


---

## 6. Next Steps

The following steps are ordered by dependency. Step 1 unblocks all remaining work.

1. Receive `calib.json` and `config.json`
2. Place both files in the repository config directory (location to confirm with supervisor)
3. Update the launch file to pass correct paths for `calibration_file_path` and `configuration_file_path`
4. Rebuild and relaunch — verify `TrackMonocular()` no longer crashes
5. Confirm Basalt processes the first frame successfully and produces a pose output
6. Verify pose is published: `ros2 topic echo /basalt/pose`
7. Validate that a feature map is being built alongside the pose estimates
8. Open pull request for `vio_sim.launch.py` on branch `feature/vio-sim-launch`
9. Begin design of the planning and policy layer — next phase of the research contribution

---

## 7. Session Log

Each session is recorded here as a reference for future contributors and to support the git commit history for the research paper.

---

### Session 1 — Architecture Study

**Title:** Study codebase architecture and trace full VIO pipeline

**Problem:** No prior understanding of how the codebase was structured or how data was intended to flow from the Gazebo simulation into the Basalt algorithm. A clear mental model was needed before any implementation work could begin.

**Solution:** Read `architecture.md`, `vio.md`, and the `README.md` files in both the tello and basalt directories. Traced the complete data flow from Gazebo camera through ROS2 topics into the Basalt node. Identified the role of each source file, the three required startup parameters, and the expected image topic name.

**Files changed:** None — study session only.

---

### Session 2 — Basalt Build Verification

**Title:** Verify Basalt SLAM node builds and starts correctly from source

**Problem:** It was unknown whether the Basalt node could be compiled and run in the current workspace before attempting any launch file integration. A non-building node would have blocked all subsequent work.

**Solution:** Built `basalt_slam_node` from source by compiling `driver.cpp`, `node.cpp`, and `slam.cpp` together. Ran the resulting executable and confirmed the expected startup message appeared. No linker errors or missing dependencies were encountered.

**Files changed:** None — build verification only.

---

### Session 3 — Launch File Creation

**Title:** Create vio_sim.launch.py to start simulation and Basalt node together

**Problem:** No launch file existed to run the Tello Gazebo simulation and the Basalt SLAM node in a single command. The two systems needed to be connected with the correct parameters wired between them.

**Solution:** Created `tello_driver/launch/vio_sim.launch.py` using `IncludeLaunchDescription` to pull in the existing simulation launch file and `ExecuteProcess` to start `basalt_slam_node` with `camera_topic_name` passed as a ROS2 parameter argument. Calibration and configuration file paths included as placeholder arguments pending supervisor delivery.

**Files changed:** `tello_driver/launch/vio_sim.launch.py`

---

### Session 4 — Simulation Debugging

**Title:** Fix shared library errors preventing the Gazebo simulation from launching

**Problem:** Running `sim.launch.py` produced two runtime errors that prevented the simulation from starting. First, `libTelloPlugin.so` was not found by Gazebo. Second, `libtello_msgs__rosidl_typesupport_cpp.so` was not found at runtime, preventing the Tello driver from loading.

**Solution:** Exported the correct build directory to `GAZEBO_PLUGIN_PATH` to make `libTelloPlugin.so` discoverable. Added the tello_msgs install path to `LD_LIBRARY_PATH` to resolve the second missing library. After both fixes the simulation launched successfully, the drone spawned in Gazebo, and the Tello plugin attached correctly to the drone model.

**Files changed:** Environment variable configuration only. No source files modified.

---

### Session 5 — Camera Root Cause Investigation

**Title:** Identify why /drone1/image_raw was not being published by the simulation

**Problem:** After the simulation launched and the drone spawned, `/drone1/image_raw` did not appear in `ros2 topic list`. The Basalt SLAM node was running but receiving no data.

**Solution:** Investigated Gazebo terminal logs and identified the root cause as a missing X11 display in the Docker container. Without access to a rendering context, Gazebo disabled its rendering subsystem entirely, preventing the `CameraSensor` from being created. The fix required X11 forwarding or a virtual display to be configured in the Docker environment.

**Files changed:** None — investigation only.

---

### Session 6 — Image Pipeline Verified, Calibration Blocker Identified

**Title:** Confirm GrabImage() receives simulator frames and identify TrackMonocular() crash cause

**Problem:** Following the X11 display fix, it was necessary to confirm that the full pipeline from Gazebo camera to SLAM node entry point was operational, and to identify why the node was not producing pose output.

**Solution:** Verified that Gazebo publishes `/drone1/image_raw` successfully. Confirmed the Basalt lifecycle node activates, creates its image subscriber, and that `GrabImage()` is called continuously as frames arrive from the simulator. Identified that the crash occurs inside `TrackMonocular()` immediately on the first frame, caused by `calibration_file_path` and `configuration_file_path` both being empty strings. The image pipeline itself is fully functional. The remaining blocker is purely the missing calibration and configuration files.

**Files changed:** None — verification and investigation only.

---
