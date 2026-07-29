# The `basalt::Controller` API as consumed by `BasaltSLAM`

**Date:** 2026-07-25. **Files:** `src/basalt/slam.cpp`, `src/basalt/node.cpp`, `include/slam/basalt/{slam.h,node.hpp}`, `ext/basalt/include/basalt/controller.h`, `ext/basalt/include/basalt/optical_flow/optical_flow.h`.

This records the exact vendored-basalt surface the `slam` package depends on, so a basalt submodule bump can be checked against it without re-reading the headers.

## `basalt::Controller` — the consumed surface

From `ext/basalt/include/basalt/controller.h`:

```cpp
namespace basalt {

enum class SlamMode { VO, VIO };                                              // :17

class Controller {
public:
    Controller(const std::string& config_path, const std::string& calib_path,
               SlamMode mode);                                                // :21-22

    void initialize();                                                        // :33  (default biases)

    void initialize(int64_t t_ns, const Sophus::SE3d& T_w_i,
                    const Eigen::Vector3d& vel_w_i, const Eigen::Vector3d& bg,
                    const Eigen::Vector3d& ba,
                    bool useProducerConsumerArchitecture = false,
                    bool enableVisualisation = false);                        // :38-43

    void GrabIMU(basalt::ImuData<double>::Ptr data);                          // :46

    void TrackMonocular(OpticalFlowInput::Ptr& frame, Sophus::SE3f& tcw,
                        std::optional<Sophus::SE3d> gtcw = std::nullopt);     // :56-57

    basalt::Calibration<double>& GetCalibration();                            // :62

    void Stop();                                                              // :28
};

}  // namespace basalt
```

Notes that matter at the call sites:

- **`TrackMonocular` takes a non-const lvalue reference** to the `Ptr`. The argument must be a named variable, not a temporary.
- **`GetCalibration()` returns a mutable reference** to the controller's `Calibration<double> calib_` member, which is held **by value** (`controller.h:71`). That by-value member is the type whose layout diverges under mismatched `-march` — see [`build-and-abi.md`](build-and-abi.md).
- The seven-argument `initialize` overload is the one used; the trailing `enableVisualisation` flag is the single source of truth for whether the GUI is required.

## `OpticalFlowInput` — a plain aggregate

From `ext/basalt/include/basalt/optical_flow/optical_flow.h:51-56`:

```cpp
struct OpticalFlowInput {
    using Ptr = std::shared_ptr<OpticalFlowInput>;

    int64_t t_ns;
    std::vector<ImageData> img_data;
};
```

No user-declared constructor and **no camera-count argument**. Construct with `std::make_shared<basalt::OpticalFlowInput>()` — note the parentheses go *after* the template argument list. Writing `std::make_shared<basalt::OpticalFlowInput()>` is ill-formed: `OpticalFlowInput()` in that position names a **function type** ("function taking no arguments returning `OpticalFlowInput`"), not the class, and the call parentheses are missing entirely. That exact typo was a build blocker in July 2026.

`Ptr` is a `shared_ptr`, so a default-constructed `OpticalFlowInput::Ptr` is **null**, not an empty object. Dereferencing one without allocating is a guaranteed segfault on the first frame.

## `eSLAMType` → `basalt::SlamMode` mapping

The mapping goes through a string, hopping between three files:

| `eSLAMType` (`include/slam/slam.hpp:120`) | string (`src/basalt/node.cpp:87-95`) | `basalt::SlamMode` (`src/basalt/slam.cpp:21-25`) |
|---|---|---|
| `VSLAM` | `"monocular-only"` | `SlamMode::VO` |
| `VISLAM` | `"monocular-inertial"` | `SlamMode::VIO` |

The intermediate string buys nothing and is what let the two ends drift apart historically. A direct `eSLAMType` → `SlamMode` conversion would be an improvement; recorded here rather than done, because it changes `BasaltSLAM`'s constructor signature.

`/ws/djinn:270-282` launches with `slam-type:=VISLAM` and `imu-topic:=/ap/imu/experimental/data`, so **VIO is the intended SITL mode**. The configuration file it passes, `ext/basalt/data/sitl_config_vo.json`, is a full VIO config despite the `_vo` suffix — the filename is a naming artefact, not a mode selector.

## Topic QoS the node must request

`BasaltSLAMNode::on_activate` creates two subscriptions. Their QoS is **not** interchangeable:

| Subscription | Publisher offers | Must request |
|---|---|---|
| camera (`node.cpp:100-102`) | `image_transport::advertise(topic, 1)` from `airsim_ros_wrapper.cpp:265` ⇒ VOLATILE, RELIABLE, KEEP_LAST(1) | a bare depth is fine (RELIABLE ≥ RELIABLE) |
| IMU (`node.cpp:105-107`) | AP_DDS ⇒ VOLATILE, **BEST_EFFORT**, KEEP_LAST(5) | **`rclcpp::SensorDataQoS()`** — a bare depth requests RELIABLE and will **never match**, so `GrabIMU` is never called and VIO silently runs with no IMU |

Full matrix and the DDS matching rule: [`/ws/ros_ws/context/ros-qos-compatibility.md`](../../../context/ros-qos-compatibility.md); publisher-side table: [`/ws/context/ardupilot-dds-qos.md`](../../../../context/ardupilot-dds-qos.md).

## Other cross-boundary types

`include/slam/slam.hpp:81-88` defines `Imu::toBasaltImuData()` inline, returning a `basalt::ImuData<double>::Ptr` constructed in the **parent** translation unit and consumed inside `libbasalt.so`. `ImuData<Scalar>` (`ext/basalt/thirdparty/basalt-headers/include/basalt/imu/imu_types.h:254-268`) holds `int64_t t_ns` plus two `Eigen::Matrix<Scalar,3,1>` — 24 bytes each, not a multiple of 16, therefore **not** fixed-size vectorizable and immune to the alignment divergence. It is safe by accident, not by design; with the ABI pin in place it is safe by construction.

`Sophus::SE3d` is the other boundary crosser: `src/basalt/slam.cpp:37` constructs one in the parent TU and passes it by const reference into `Controller::initialize`. It contains a `Quaterniond` (32 B, fixed-size vectorizable) and so **does** diverge — 16-byte aligned in the node, 32-byte in basalt — under mismatched `-march`. `Sophus::SE3f` happens to be safe (16-byte `Quaternionf`), which is the kind of accidental survival that makes these bugs intermittent.

## Initialisation ordering

`BasaltSLAM::InitialiseSlam` (`src/basalt/slam.cpp:19-45`) constructs the `Controller`, reads the calibrated gyro/accel biases back out of it via `GetCalibration()`, and calls the seven-argument `initialize` **once**, at construction. It is not called per frame — an earlier revision had the `initialize` call inside `TrackMonocular`, which re-initialised the estimator on every image.
