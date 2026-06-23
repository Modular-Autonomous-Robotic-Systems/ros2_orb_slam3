# VIO-SLAM Integration — Progress Tracker

**Author:** Taanvi Arora
**Project:** Neurorobotics Workspace — Autonomous Drone with Visual Inertial Odometry
**Repo:** https://github.com/Modular-Autonomous-Robotic-Systems/neurorobotics-ws
**Started:** June 2026 | **Last Updated:** June 23, 2026

---

## What We Are Building

```
[Gazebo Simulation]
       │  libgazebo_ros_camera.so
       ▼
[/drone1/image_raw]            ← ROS2 image topic ~25 Hz
       │
       ▼
[basalt_slam_node — node.cpp]  ← subscribes, calls GrabImage()
       │
       ▼
[slam.cpp]                     ← converts frame, calls TrackMonocular()
       │
       ▼
[Basalt Core VIO]              ← pose estimation + map
       │
       ▼
[Planning / Policy Layer]      ← FUTURE: exploration policy → paper
```

**Key files:**
- `driver.cpp` → entry point, builds executable `basalt_slam_node`
- `node.cpp` → ROS2 layer: params, subscriber, `GrabImage()` callback
- `slam.cpp` → Basalt wrapper: frame conversion, `TrackMonocular()`

**Parameters the Basalt node needs:**

| Parameter | Value |
|---|---|
| `camera_topic_name` | `/drone1/image_raw` |
| `calibration_file_path` |
| `configuration_file_path` | 

---

## Overall Status 

| Task | Status |
|---|---|
| Tello simulation (`sim.launch.py`) | ✅ Done|
| Codebase reading (architecture.md,vio.md,README file) | ✅ Done |
| Basalt node build verified | ✅ Done |
| Tello plugin loading fixed | ✅ Done |
| tello_msgs dependency fixed | ✅ Done |
| Drone spawn in Gazebo verified | ✅ Done |
| Launch file (`vio_sim.launch.py`) | ✅ Done |
| Docker/X11 fix for Gazebo camera | 🔴 Blocker|
| `/drone1/image_raw` publishing | 🔴 Blocked |
| `GrabImage()` verification | 🔴 Blocked |
| Calibration integration |
| Pose output verification |
| PR for `vio_sim.launch.py` |

---

## Session Log

### Session 1 — Architecture Study

**What I did:** Read `architecture.md`, `vio.md`, `README.md` in tello and basalt directories. Traced the full pipeline from Gazebo to Basalt.

**What I found:**
- `driver.cpp` = entry point, builds `basalt_slam_node`
- `node.cpp` = ROS2 layer, image subscriber, calls `GrabImage()`
- `slam.cpp` = wrapper above Basalt, calls `TrackMonocular()`
- Three required params: `camera_topic_name`, `calibration_file_path`, `configuration_file_path`
- Expected image topic: `/drone1/image_raw`

---

### Session 2 — Basalt Build Verification

**What I did:** Built the Basalt SLAM node from source. Confirmed all three files compile into one executable.

**Result:**
```
./build/slam/basalt_slam_node
→ Initialising Slam Node
```

---

### Session 3 — Launch File Creation

**What I did:** Created `tello_driver/launch/vio_sim.launch.py`.

**What it does:**
- Includes `sim.launch.py` via `IncludeLaunchDescription`
- Launches `basalt_slam_node`

from launch import LaunchDescription 
from launch.actions import ExecuteProcess, IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def generate_launch_description():
 
	return LaunchDescription([ 
		IncludeLaunchDescription( 
			PythonLaunchDescriptionSource( 
				"/ws/ros_ws/src/tello/tello_driver/launch/sim.launch.py" 
			) 
		), 
		ExecuteProcess( 
		cmd=[ 
			"/ws/ros_ws/build/slam/basalt_slam_node", 
			"--ros-args", 
			"-p", "camera_topic_name:=/drone1/image_raw" 
		], 
			output="screen" 
	) 
])
### Session 4 — Simulation Debugging

**What I did:** Ran `sim.launch.py`, hit errors, fixed them one by one.

**Problem 1: `libTelloPlugin.so not found`**
- Cause: `GAZEBO_PLUGIN_PATH` not set correctly
- Fix: exported correct build path
- Status: ✅ Fixed

**Problem 2: `libtello_msgs__rosidl_typesupport_cpp.so not found`**
- Cause: `LD_LIBRARY_PATH` missing tello_msgs
- Fix: added tello_msgs install path
- Status: ✅ Fixed

**Result after fixes:**
```
TELLO PLUGIN
-----------------------------------------
link_name: base_link_1
Successfully sent factory message to Gazebo
```

---

### Session 5 — Camera Root Cause Investigation

**What I did:** Investigated why `/drone1/image_raw` wasn't in `ros2 topic list`. Checked Gazebo logs carefully.

**Root cause:**
```
Can't open display :0
Authorization required
→ Rendering disabled
→ CameraSensor: Failed to create
→ /drone1/image_raw never published
→ GrabImage() never called
```


## Phase Checklist

### Phase 0 — Codebase Understanding ✅ COMPLETE
Read `architecture.md`, `vio.md`, READMEs
Identify executable: `basalt_slam_node`
Identify image callback: `GrabImage()` in `node.cpp`
Identify camera plugin: `libgazebo_ros_camera.so`
Identify image topic: `/drone1/image_raw`
Confirm parameter names: `camera_topic_name`, `calibration_file_path`, `configuration_file_path`

### Phase 1 — Simulation Environment ✅ MOSTLY COMPLETE
`sim.launch.py` runs Gazebo
Drone spawns in world
Fixed `GAZEBO_PLUGIN_PATH` → Tello plugin loads
Fixed `LD_LIBRARY_PATH` → tello_msgs loads
Tello plugin confirms: `link_name: base_link_1`
🔴 Fix Docker/X11 → camera sensor needs display to render

### Phase 2 — Basalt Node Build ✅ COMPLETE
`driver.cpp` + `node.cpp` + `slam.cpp` build into `basalt_slam_node`
Node starts: `Initialising Slam Node` confirmed
No linker or runtime errors

### Phase 3 — Launch File IMPLEMENTED
`vio_sim.launch.py` created
Includes `sim.launch.py`
Launches `basalt_slam_node` with all three parameters
Confirm exact ROS2 package name from `CMakeLists.txt`

### Phase 4 — Image Pipeline Verification 🔴 BLOCKED
Fix Docker/X11 (unblocks everything below)
Confirm `/drone1/image_raw` in `ros2 topic list`
Confirm subscription: `ros2 topic info /drone1/image_raw`
Add log in `GrabImage()` in `node.cpp`:
  ```cpp
  RCLCPP_INFO(this->get_logger(), "[Basalt] IMAGE RECEIVED, stamp: %ld",
               msg->header.stamp.nanosec);
  ```
Rebuild + relaunch — confirm log fires at ~25 Hz

### Phase 5 — Calibration Integration ⏳ WAITING ON SUPERVISOR
Receive `calib.json` + `config.json`
Place in repo config directory
Update launch file default paths
Verify SLAM node initializes without errors
Check pose output: `ros2 topic echo /basalt/pose`

### Phase 6 — PR and Docs
`PROGRESS.md` maintained
`THEORY_NOTES.md` written
Create branch `feature/vio-sim-launch`
Open PR: `Add VIO simulation launch file integrating Basalt SLAM with Tello sim`

---

## Current Pipeline State

```
[Gazebo Simulation]    ✅ Launches
        ↓
[Drone Spawn]          ✅ tello_1 spawned
        ↓
[Tello Plugin]         ✅ Loads — link_name: base_link_1
        ↓
[CameraSensor]         🔴 FAILS — no X11 display in Docker
        ↓
[/drone1/image_raw]    🔴 Not published
        ↓
[basalt_slam_node]     ✅ Starts (waiting for image data)
        ↓
[GrabImage()]          🔴 Never called
        ↓
[slam.cpp / Basalt]    ⏳ Pending
        ↓
[Pose Output]          ⏳ Pending calibration
```