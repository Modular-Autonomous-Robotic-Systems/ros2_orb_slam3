# Candidate Gazebo Environments for Tello VIO/SLAM Testing

## 1. Purpose

Once SLAM is confirmed working with the corrected pinhole calibration (see `CameraCalibration.md`, Section 6), the drone-only Gazebo world needs to be replaced with something that has structure to localize against: walls, furniture, terrain, trees. This document lists the environments found, grouped by complexity, with notes on how each would integrate with the current `tello_ros` stack.

A compatibility note up front: the current setup uses `libgazebo_ros_camera.so`, which is the plugin naming convention for classic Gazebo (Gazebo 9/11), not the newer Gazebo Sim (Harmonic/Ionic, formerly Ignition). Several of the options below only ship for the newer Gazebo Sim. This is flagged per entry as it affects how much integration work is required.

The current simulation launches the Tello in a largely empty Gazebo world, which provides insufficient visual structure for evaluating long-term visual odometry and SLAM performance.

## 2. Tier 1 - Minimal / Build-Your-Own

Lowest complexity, most control. Individual assets (walls, crates, simple furniture) pulled from a model library and hand-placed into a small custom world.

- Gazebo Fuel model library (app.gazebosim.org/fuel/models) - hundreds of individual models, includable directly into a custom `.world` SDF file via `<include><uri>` tags. Works with both classic Gazebo and Gazebo Sim, but the newer Fuel-hosted models increasingly target Gazebo Sim.
- `leonhartyao/gazebo_models_worlds_collection` (GitHub) - a pre-aggregated set of classic-Gazebo worlds and models pulled from several public projects. Drop-in via `GAZEBO_MODEL_PATH` / `GAZEBO_RESOURCE_PATH`.

Good for: fastest path to "not an empty world," full control over feature density, but requires manually building the scene.

## 3. Tier 2 - Indoor, Structured (walls, rooms, furniture)

Closest match to the Tello's actual use case (small indoor flights) and to what was asked for (walls, houses).

- `aws-robotics/aws-robomaker-small-house-world` (GitHub, `ros2` branch) - a multi-room house with furniture, explicit ROS2 launch files (`small_house.launch.py`), classic Gazebo. Likely the best first choice: closest to a real indoor test, ROS2-native, and low integration effort since it already ships a launch file to include into `sim.launch.py`.
- `aws-robotics/aws-robomaker-small-warehouse-world` (GitHub, `ros2` branch) - shelving, pallets, clutter. Same integration path as the house world. Useful as a second, more feature-dense indoor test.
- `mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps` (GitHub) - a small house world plus several other worlds/maps originally built for Nav2 testing; includes some worlds with dynamic obstacles.

Good for: matches the Tello's real deployment context, ROS2 launch files already exist, classic-Gazebo compatible.

## 4. Tier 3 - Outdoor, Vegetation / Terrain (trees)

- `ctu-mrs/mrs_gazebo_common_resources` (GitHub) - part of the CTU Multi-Robot Systems UAV simulation stack; ships a ready-made `forest.sdf` world plus reusable `tree_simple` and `grass_plane` models. Purpose-built for drone simulation rather than ground robots, and classic-Gazebo based, which matches the current stack.
- `ethz-asl/forest_gen` (GitHub) - a generator (not a fixed world) that produces randomized Poisson-distributed forest worlds parameterized by map size and tree density, originally built for UAV collision-avoidance evaluation. Useful if a range of tree densities is wanted for benchmarking rather than one fixed scene.
- `kubja/gazebo-vegetation` (GitHub) - a standalone package of tree and bush models (not a full world) that can be inserted into any custom world, similar in spirit to the Tier 1 approach but vegetation-specific.

Good for: outdoor obstacle density testing, matches "trees" requirement directly, all classic-Gazebo compatible and drone-oriented (the CTU-MRS and ethz-asl repos were both built for UAV work, not ground robots).

## 5. Tier 4 - Large-Scale Outdoor / Airport-Scale

- PX4's world set (`PX4/PX4-gazebo-models/tree/main/worlds`, formerly `PX4/sitl_gazebo/worlds`) - includes `baylands`, `ksql_airport`, `mcmillan_airfield`, `sonoma_raceway`, `warehouse`, `yosemite`, and others. Large, higher fidelity, some outdoor/airport scale.
- `larics_gazebo_worlds` (GitHub) - custom UAV-focused worlds from the LARICS lab.

Caveat: the current PX4 world set targets the newer Gazebo Sim (Harmonic), or requires the full PX4 SITL toolchain for the classic-Gazebo versions. Since the current stack is `tello_ros` on classic Gazebo without PX4, this tier has the highest integration cost and is probably not worth pursuing unless the project later migrates toward PX4 SITL or Gazebo Sim.

## 6. Recommendation for a First Pass

Based on compatibility with the current ROS 2 + classic Gazebo setup and the expected effort required for integration, these two environments are recommended for initial testing.

1. `aws-robomaker-small-house-world` (indoor, walls/furniture, ROS2 branch) - swap in as the primary SLAM benchmark, since it is closest to a realistic indoor Tello flight.
2. `ctu-mrs/mrs_gazebo_common_resources` `forest.sdf` (outdoor, trees) - second benchmark for an outdoor/vegetated scene, already built for drone simulation.

Both are good candidates for future integration into the current tello_ros workspace. A likely integration approach would be to clone the repositories into src/, install any dependencies, rebuild the workspace, and update sim.launch.py (or an included launch file) to load the desired world. The exact integration steps would depend on the selected environment and its launch configuration.

## 7. Sources

- https://github.com/aws-robotics/aws-robomaker-small-house-world (ros2 branch)
- https://github.com/aws-robotics/aws-robomaker-small-warehouse-world (ros2 branch)
- https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps
- https://github.com/ctu-mrs/mrs_gazebo_common_resources
- https://github.com/ethz-asl/forest_gen
- https://github.com/kubja/gazebo-vegetation
- https://github.com/leonhartyao/gazebo_models_worlds_collection
- https://app.gazebosim.org/fuel/models
- https://docs.px4.io/main/en/sim_gazebo_gz/worlds (PX4/PX4-gazebo-models)
- https://github.com/larics/larics_gazebo_worlds
