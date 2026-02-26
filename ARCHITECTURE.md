# SLAM Package Architecture

## Overview

The `slam` package provides a flexible ROS 2 wrapper for various Visual SLAM (Simultaneous Localization and Mapping) algorithms. It is designed to decouple the ROS 2 interface from the underlying SLAM implementation, allowing for easy switching between different backends (currently ORB-SLAM3 and MORB_SLAM).

## Core Components

The architecture consists of three main layers:
1.  **ROS 2 Interface Layer**: Handles data ingestion (images, IMU) and publication (pose, point clouds, TF).
2.  **Abstraction Layer**: Defines a generic interface for SLAM systems, ensuring the core algorithm is independent of ROS.
3.  **Implementation Layer**: Concrete wrappers for specific SLAM libraries.

### 1. ROS 2 Interface Layer

*   **`SlamNode` (`include/slam/node.hpp`)**:
    *   **Description**: Base class inheriting from `rclcpp_lifecycle::LifecycleNode`. It manages common ROS functionalities like TF broadcasting and lifecycle state transitions.
    *   **Key Responsibilities**:
        *   Managing the `tf2_ros::TransformBroadcaster`.
        *   Publishing the estimated camera pose as a `geometry_msgs::msg::TransformStamped` to the `/tf` topic.
    *   **Key Methods**:
        *   `void Update()`: Called after every frame is processed by the underlying SLAM algorithm to update node state and broadcast estimated transform.
        *   `void PublishPositionAsTransform(Sophus::SE3f &Tcw)`: Helper to broadcast the transform message.

*   **`VisualSlamNode` (`include/slam/monocular.hpp`)**:
    *   **Description**: Inherits from `SlamNode` and specializes in monocular visual SLAM.
    *   **Key Responsibilities**:
        *   Subscribing to camera images (`sensor_msgs::msg::Image`) and converting them.
        *   Instantiating the concrete SLAM backend (e.g., `MonoORBSLAM3`) based on configuration.
        *   Publishing visualization data (annotated frames, map points).
    *   **Key Methods**:
        *   `void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)`: Callback for new images. Converts ROS image to `Frame` and calls `mpSlam->TrackMonocular`.
        *   `void PublishFrame()`: Publishes the debug image with tracked features.
        *   `void PublishMapPointsCallback(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw)`: Callback from the SLAM engine to publish the sparse map.

### 2. Abstraction Layer

This layer ensures that the actual SLAM algorithm is implemented independently of the ROS 2 interface, allowing for easy substitution or modification of the underlying engine without affecting the ROS node structure.

*   **`Slam` (`include/slam/slam.hpp`)**:
    *   **Description**: The abstract base class (interface) for all SLAM implementations. It defines the standard contract for feeding data into the SLAM algorithm and retrieving results.
    *   **Key Responsibilities**: Providing a uniform API for tracking and system control.
    *   **Key Methods**:
        *   `virtual cv::Mat GetCurrentFrame() = 0`: Returns the current frame, potentially with debug annotations.
        *   `virtual void TrackMonocular(Frame &frame, Sophus::SE3f &tcw) = 0`: Processes a single frame. Returns the estimated World-to-Camera transform (`tcw`) via reference.
        *   `virtual void Shutdown() = 0`: Signal to the SLAM system to stop threads and save maps.
        *   `virtual void SetFrameMapPointUpdateCallback(std::function<void(std::vector<ORB_SLAM3::MapPoint*>&, const Sophus::SE3<float>&)> callback) = 0`: Sets a callback for receiving map point updates from the SLAM algorithm.

*   **`Frame` (`include/slam/slam.hpp`)**:
    *   **Description**: A simple data structure used to encapsulate frame data required by the SLAM algorithm, specifically the image and its timestamp.
    *   **Key Responsibilities**: Holding the image data (`cv::Mat`) and timestamp in a thread-safe manner (via `std::shared_ptr`).
    *   **Key Methods**:
        *   `cv::Mat getImage()`: Returns the image matrix.
        *   `double getTimestampSec()`: Returns the timestamp in seconds.
        *   `long getTimestampNSec()`: Returns the timestamp in nanoseconds.
        *   `Sophus::SE3f getTransform()`: Returns the pose of the frame (`Tcw`).

### 3. Implementation Layer

*   **`MonoORBSLAM3` (`include/slam/orbslam3/monocular.hpp`)**:
    *   **Description**: Concrete implementation of the `Slam` interface that wraps the `ORB_SLAM3::System`.
    *   **Key Responsibilities**: Translating `Frame` objects into `ORB_SLAM3` native calls and managing the ORB-SLAM3 system lifecycle.
    *   **Key Methods**:
        *   `void TrackMonocular(Frame &frame, Sophus::SE3f &tcw) override`: Invokes `mpORBSlam3->TrackMonocular(frame.getImage(), frame.getTimestampSec())` to perform tracking.
        *   `void SetFrameMapPointUpdateCallback(std::function<void(std::vector<ORB_SLAM3::MapPoint*>&, const Sophus::SE3<float>&)> callback) override`: Passes the callback to the underlying `ORB_SLAM3::System`.

*   **`MonoMORBSLAM` (`include/slam/morbslam/monocular.hpp`)**:
    *   **Description**: Concrete implementation for `MORB_SLAM`, following the same pattern as `MonoORBSLAM3`.

### 4. Driver / Entry Points

The driver files contain the `main` function, serving as the entry point for the ROS node executable. They are responsible for instantiating the node class and spinning the ROS executor.

*   **`src/monocular_driver.cpp`**:
    *   **Role**: Implements the main entry point for the monocular SLAM node.
    *   **Function**: `int main(int argc, char **argv)`
    *   **Operation**:
        1.  Initializes ROS 2 (`rclcpp::init(argc, argv)`).
        2.  Instantiates `VisualSlamNode` (`auto node = std::make_shared<VisualSlamNode>()`).
        3.  Spins the node (`rclcpp::spin(node->get_node_base_interface())`) to process callbacks (e.g., `GrabImage`).
        4.  Shuts down ROS 2 on exit (`rclcpp::shutdown()`).

## Data Flow

The data flow describes the sequence of operations from receiving an image to publishing the estimated pose and map data.

1.  **Image Reception and Conversion**:
    *   A ROS 2 `sensor_msgs::msg::Image` message is received by the `VisualSlamNode`'s subscriber, triggering the callback:
        *   `void VisualSlamNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)`
    *   Inside `GrabImage`:
        *   The ROS image message (`msg`) is converted into an OpenCV `cv::Mat` using `cv_bridge::toCvCopy(msg)`.
        *   A `Frame` object (`mpCurrentFrame`) is created using its constructor:
            *   `Frame::Frame(std::shared_ptr<cv::Mat> image, long &timestamp)`

2.  **SLAM Processing**:
    *   Still within `VisualSlamNode::GrabImage`, the created `Frame` is passed to the abstract `Slam` interface's `TrackMonocular` method:
        *   `mpSlam->TrackMonocular(mpCurrentFrame, tcw)`
    *   This is a polymorphic call, which dispatches to the concrete implementation (e.g., `MonoORBSLAM3::TrackMonocular`):
        *   `void MonoORBSLAM3::TrackMonocular(Frame &frame, Sophus::SE3f &tcw)`
    *   Inside `MonoORBSLAM3::TrackMonocular`:
        *   The underlying ORB_SLAM3 system's tracking function is called:
            *   `mpORBSlam3->TrackMonocular(frame.getImage(), frame.getTimestampSec())`
        *   The estimated World-to-Camera transform (`tcw`) is returned by ORB_SLAM3 and stored in the `tcw` reference parameter.

3.  **Output and Publication**:
    *   After `mpSlam->TrackMonocular` returns, back in `VisualSlamNode::GrabImage`:
        *   The World-to-Camera transform (`tcw`) is inverted to obtain the Camera-to-World transform (`mpTwc = tcw.inverse()`).
        *   The `VisualSlamNode::Update()` method is invoked:
            *   `void VisualSlamNode::Update()`
    *   Inside `VisualSlamNode::Update()`:
        *   The base class `SlamNode::Update()` is called:
            *   `void SlamNode::Update()`
        *   Inside `SlamNode::Update()`, the pose is broadcast via TF:
            *   `void SlamNode::PublishPositionAsTransform(Sophus::SE3f &tcw)` (using `mpTwc`).
        *   `void VisualSlamNode::PublishFrame()` is called to publish the annotated image.
    *   Additionally, map point updates are handled via a callback mechanism. When `mpSlam->SetFrameMapPointUpdateCallback()` is set during initialization, the SLAM algorithm (e.g., ORB_SLAM3) will periodically invoke:
        *   `void VisualSlamNode::PublishMapPointsCallback(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw)`
        *   This function converts the map points to `sensor_msgs::msg::PointCloud2` and publishes them.

## Directory Structure

*   `include/slam/`: Header files for the ROS node and abstractions.
    *   `node.hpp`: Definition of the base `SlamNode` class.
    *   `monocular.hpp`: Definition of the `VisualSlamNode` class.
    *   `slam.hpp`: Definitions of the abstract `Slam` interface and `Frame` data structure.
    *   `orbslam3/`: Contains ORB-SLAM3 specific headers.
        *   `monocular.hpp`: Definition of `MonoORBSLAM3` wrapper.
    *   `morbslam/`: Contains MORB-SLAM specific headers.
        *   `monocular.hpp`: Definition of `MonoMORBSLAM` wrapper.
*   `src/`: Source files for the ROS node and wrappers.
    *   `monocular_driver.cpp`: **Entry point**. Contains the `main` function to launch the monocular node.
    *   `monocular.cpp`: Implementation of `VisualSlamNode`.
    *   `node.cpp`: Implementation of `SlamNode`.
    *   `slam.cpp`: Implementation of `Slam` base methods and helper functions (e.g., `readYAMLFile`, `Frame` constructor).
    *   `orbslam3/`: Contains ORB-SLAM3 specific source files.
        *   `monocular.cpp`: Implementation of `MonoORBSLAM3`.
    *   `morbslam/`: Contains MORB-SLAM specific source files.
        *   `monocular.cpp`: Implementation of `MonoMORBSLAM`.
*   `orb_slam3/`: The source code for the ORB-SLAM3 library (built as an internal shared library). This directory contains the original ORB-SLAM3 project structure including its `src`, `include`, and `Thirdparty` subdirectories.
*   `ext/`: External submodules (e.g., Basalt, Pangolin) that are used by the overall workspace but might not be directly used by this specific `slam` package's ROS nodes.
