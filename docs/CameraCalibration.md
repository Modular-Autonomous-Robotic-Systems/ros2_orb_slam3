# Camera Calibration: Models, JSON Schema, and Ingestion Pipeline

## 1. Introduction

### 1.1 Motivation

Accurate camera calibration is the foundation of any visual-inertial odometry system. Before a single pixel can be tracked across frames or a landmark triangulated in 3D space, the imaging geometry of every physical camera must be characterised precisely. In the `basalt` VIO framework this characterisation is encoded in a calibration JSON file that is read once at system start-up. The contents of that file determine which camera model is instantiated, what intrinsic parameters are loaded, and how every pixel observation is subsequently projected and unprojected throughout the entire SLAM pipeline.

This document provides a complete technical reference for the calibration subsystem in `basalt`, covering:

- The catalogue of camera models available in the codebase and the JSON `camera_type` string that selects each one, together with the mathematical concepts each model is built on.
- The intrinsic parameter keys required by each model inside the `"intrinsics"` object of the calibration JSON.
- The data structures and serialization path through which the JSON file is ingested into the `Controller` and ultimately consumed by `TrackMonocular()`.
- The configuration of the Tello drone's Gazebo-simulated camera, including the camera plugin's XML parameters, and its mapping to the appropriate calibration model.
- The construction and placement of the `tello_calib.json` pinhole calibration file used for the Tello Gazebo simulation.

This document is intended as a companion to `doc/VIO.md` and `doc/LocalMapper.md`. It is oriented towards contributors who need to extend the system to new camera hardware, integrate a physical drone, or verify that a simulation calibration correctly matches a real device.

### 1.2 Scope

In scope:
- All camera models defined under `thirdparty/basalt-headers/include/basalt/camera/`, along with an introduction to the projection and distortion concepts that underlie them.
- The JSON schema for calibration files, including required keys for each model.
- The C++ ingestion pipeline from `src/node.cpp` through `Controller` (`include/basalt/controller.h`, `src/controller.cpp`) to `TrackMonocular()`.
- The Tello Gazebo camera configuration derived from `tello_description/urdf/tello.xml`, including the relevant parameters of the `libgazebo_ros_camera.so` plugin.
- A discussion of the current simulation calibration, the appropriate choice of camera model, and the resulting `tello_calib.json` file.

Out of scope:
- The full mathematical derivation of each projection function (refer to the original camera model papers listed in Section 8).
- Camera-IMU extrinsic calibration (covered separately).
- Online camera re-calibration or auto-calibration.

Section 2 introduces the general concepts of camera modelling and catalogues all camera models and their `camera_type` strings. Section 3 defines the full JSON schema. Section 4 traces the ingestion pipeline from launch file to `TrackMonocular()`. Section 5 documents the Tello Gazebo camera configuration and plugin parameters. Section 6 discusses the choice of camera model for the Tello simulation and the resulting calibration file. Section 7 lists the key classes and interfaces referenced throughout this document. Section 8 lists references.

---

## 2. Camera Models

### 2.1 Overview

Before cataloguing the individual models, it is useful to establish the shared vocabulary and mathematics that every camera model in `basalt` builds on. All camera models in `basalt` are defined under:

```
thirdparty/basalt-headers/include/basalt/camera/
```

At runtime, a single camera model is selected via the `"camera_type"` string in the calibration JSON. This string is matched against each model's static `getName()` method inside `GenericCamera<Scalar>::visitAllTypes()` (see Section 4.5, `thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp`). The following eight models are currently implemented.

#### 2.1.1 What a Camera Model Describes

A camera model is a pair of functions that relate a 3D point observed by the sensor to the 2D pixel coordinate it produces, and vice versa:

- Projection, $\pi: \mathbb{R}^3 \rightarrow \mathbb{R}^2$, maps a 3D point $\mathbf{P} = (X, Y, Z)^T$ expressed in the camera's coordinate frame to a pixel coordinate $(u, v)$.
- Unprojection, $\pi^{-1}: \mathbb{R}^2 \rightarrow \mathbb{S}^2$, is the inverse operation: given a pixel $(u, v)$, it recovers the 3D bearing vector (a unit-length ray direction) along which the originating point must lie. Because depth is lost during projection, unprojection can only ever recover a ray, not a full 3D point.

Every model in `basalt` implements both functions, along with the analytic Jacobians of each with respect to the 3D point and the intrinsic parameters. These Jacobians are what feed the reprojection residual $\mathbf{r}_{ij}$ and its linearisation during VIO optimisation (see `doc/Marginalisation.md`, Section 3).

#### 2.1.2 Intrinsic Parameters

All eight models share a common pair of parameters describing the linear part of the projection:

- Focal length, $f_x$ and $f_y$ (pixels): the scale factors that convert a normalised ray direction into pixel units along the horizontal and vertical image axes. Physically these correspond to the distance between the lens and the image sensor, expressed in pixel units via the sensor's pixel pitch.
- Principal point, $c_x$ and $c_y$ (pixels): the pixel coordinate at which the optical axis intersects the image plane. For an ideal, centred sensor this is close to the image centre.

Together, $(f_x, f_y, c_x, c_y)$ define the linear pinhole projection described in Section 2.2. Every other model in this document augments this linear core with an additional nonlinear term that accounts for how the lens bends light before it reaches the sensor.

#### 2.1.3 Distortion

No real lens is a perfect pinhole. Lenses bend incoming light rays by an amount that depends on the angle of incidence, which displaces the pixel that a 3D point maps to relative to the ideal linear projection. This displacement is called distortion, and it is what the nonlinear parameters of each model (e.g. `k1`-`k6`, `p1`, `p2`, `xi`, `alpha`, `beta`, `w`) are fitted to capture. Distortion in `basalt`'s camera models falls into a small number of families:

- Radial distortion bends light as a function of the radial distance from the principal point. It is the dominant effect in most consumer and machine-vision lenses and is captured by polynomial coefficients ($k_1, k_2, \dots$) in models such as pinhole-radtan8 and BAL.
- Tangential distortion arises from slight misalignment between the lens and the image sensor plane and is captured by the parameters $p_1, p_2$ in the pinhole-radtan8 model.
- Sphere-based (fisheye/wide-angle) distortion is used by models designed for fields of view that a polynomial radial model cannot represent well. These models (Double Sphere, EUCM, UCM) project the 3D point onto one or more unit spheres before applying the linear pinhole step, which lets them represent field-of-view angles approaching or exceeding 180°.
- Angle-based distortion represents the mapping from the incidence angle of a ray to its radial pixel distance directly as a polynomial (Kannala-Brandt) or a single angular parameter (FOV), rather than distorting the projected pixel coordinate itself.

A calibration is therefore the process of estimating, for a specific physical (or simulated) lens and sensor, which distortion family best describes its behaviour, and what numerical values the corresponding parameters take. Selecting the wrong family for a given lens does not prevent the system from running, but it does introduce a systematic mismatch between the modelled and true imaging geometry (see Section 6.2 for the concrete case relevant to this project).

#### 2.1.4 Extrinsics

Separately from the intrinsic projection model, every camera in the calibration file carries an extrinsic transform `T_imu_cam`, the rigid-body pose of the camera relative to the IMU frame. This is a fixed calibration input (see Section 3.1) rather than part of the projection function itself, and it is out of scope for this document (Section 1.2).

With this vocabulary established, the following subsections catalogue the eight concrete models.

---

### 2.2 Pinhole (`"pinhole"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/pinhole_camera.hpp`

The classical pinhole projection model with no distortion. A 3D point $\mathbf{P} = (X, Y, Z)^T$ projects to pixel $(u, v)$ as:

$$u = f_x \frac{X}{Z} + c_x, \qquad v = f_y \frac{Y}{Z} + c_y$$

This model is exact for an ideal, undistorted lens and is the appropriate choice for the Tello Gazebo simulation, which uses zero distortion coefficients (see Section 5).

| Key | Description |
|-----|-------------|
| `fx` | Focal length along the x-axis (pixels) |
| `fy` | Focal length along the y-axis (pixels) |
| `cx` | Principal point x-coordinate (pixels) |
| `cy` | Principal point y-coordinate (pixels) |

---

### 2.3 Double Sphere (`"ds"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/double_sphere_camera.hpp`

The Double Sphere (DS) model (Usenko et al., 2018) maps the unit sphere to the image plane through two successive sphere projections, per the sphere-based distortion family introduced in Section 2.1.3. It is well-suited for fisheye and wide-angle lenses that exceed a 180 degree field of view.

| Key | Description |
|-----|-------------|
| `fx` | Focal length along x (pixels) |
| `fy` | Focal length along y (pixels) |
| `cx` | Principal point x (pixels) |
| `cy` | Principal point y (pixels) |
| `xi` | First sphere shift parameter |
| `alpha` | Blending parameter between the two spheres |

---

### 2.4 Extended Unified Camera Model (`"eucm"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/extended_unified_camera.hpp`

The EUCM model (Khomutenko et al., 2015) generalises the Unified Camera Model by adding a second distortion parameter $\beta$ that controls the shape of the projection surface, offering improved accuracy for moderately wide-angle lenses.

| Key | Description |
|-----|-------------|
| `fx` | Focal length along x (pixels) |
| `fy` | Focal length along y (pixels) |
| `cx` | Principal point x (pixels) |
| `cy` | Principal point y (pixels) |
| `alpha` | Sphere-to-plane projection parameter |
| `beta` | Shape parameter of the projection surface |

---

### 2.5 Unified Camera Model (`"ucm"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/unified_camera.hpp`

The UCM (Geyer & Daniilidis, 2000) is a degenerate case of the EUCM with $\beta = 1$. It models the camera as a combination of a sphere and a perspective projection and is widely used for omnidirectional cameras.

| Key | Description |
|-----|-------------|
| `fx` | Focal length along x (pixels) |
| `fy` | Focal length along y (pixels) |
| `cx` | Principal point x (pixels) |
| `cy` | Principal point y (pixels) |
| `alpha` | Sphere-to-plane projection parameter |

---

### 2.6 Kannala-Brandt (KB4) (`"kb4"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/kannala_brandt4_camera.hpp`

The Kannala-Brandt model (Kannala & Brandt, 2006) represents distortion as a polynomial function of the angle of incidence, per the angle-based distortion family introduced in Section 2.1.3, making it especially accurate for fisheye lenses. The `kb4` variant uses four distortion coefficients.

| Key | Description |
|-----|-------------|
| `fx` | Focal length along x (pixels) |
| `fy` | Focal length along y (pixels) |
| `cx` | Principal point x (pixels) |
| `cy` | Principal point y (pixels) |
| `k1` | Distortion coefficient 1 |
| `k2` | Distortion coefficient 2 |
| `k3` | Distortion coefficient 3 |
| `k4` | Distortion coefficient 4 |

---

### 2.7 Field-of-View (`"fov"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/fov_camera.hpp`

The FOV model (Devernay & Faugeras, 2001) approximates radial distortion with a single angle-based parameter $\omega$. It is computationally efficient and effective for moderately wide-angle lenses.

| Key | Description |
|-----|-------------|
| `fx` | Focal length along x (pixels) |
| `fy` | Focal length along y (pixels) |
| `cx` | Principal point x (pixels) |
| `cy` | Principal point y (pixels) |
| `w`  | Field-of-view distortion parameter |

---

### 2.8 Bundle Adjustment Linear (`"bal"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/bal_camera.hpp`

The BAL model uses the parameterisation from the Bundle Adjustment in the Large dataset (Agarwal et al., 2010). It encodes the camera with a single focal length and two radial distortion coefficients, intended for structure-from-motion problems where camera intrinsics are solved jointly with structure.

| Key | Description |
|-----|-------------|
| `f`  | Single focal length (pixels) |
| `k1` | Radial distortion coefficient 1 |
| `k2` | Radial distortion coefficient 2 |

---

### 2.9 Pinhole with 8-Coefficient Radial-Tangential Distortion (`"pinhole-radtan8"`)

Header path: `thirdparty/basalt-headers/include/basalt/camera/pinhole_radtan8_camera.hpp`

An extended pinhole model supporting up to eight Brown-Conrady distortion coefficients: six radial ($k_1$-$k_6$) and two tangential ($p_1$, $p_2$), corresponding to the radial and tangential distortion families introduced in Section 2.1.3. This model is compatible with the OpenCV full rational distortion model and is suitable for high-accuracy calibrations of standard lenses.

| Key | Description |
|-----|-------------|
| `fx` | Focal length along x (pixels) |
| `fy` | Focal length along y (pixels) |
| `cx` | Principal point x (pixels) |
| `cy` | Principal point y (pixels) |
| `k1`-`k6` | Radial distortion coefficients |
| `p1`, `p2` | Tangential distortion coefficients |

---

### 2.10 Summary Table

| `camera_type` string | Model name | Intrinsic count | Distortion type |
|---|---|---|---|
| `"pinhole"` | Pinhole | 4 | None |
| `"ds"` | Double Sphere | 6 | Fisheye (sphere-based) |
| `"eucm"` | Extended Unified Camera | 6 | Wide-angle (sphere-based) |
| `"ucm"` | Unified Camera | 5 | Omnidirectional (sphere-based) |
| `"kb4"` | Kannala-Brandt 4 | 8 | Fisheye (polynomial-angle) |
| `"fov"` | Field-of-View | 5 | Moderate wide-angle |
| `"bal"` | BAL | 3 | SfM radial |
| `"pinhole-radtan8"` | Pinhole + RadTan-8 | 12 | Radial-tangential (Brown-Conrady) |

---

## 3. Calibration JSON Schema

### 3.1 Top-Level Structure

The calibration file is a JSON object with the following top-level keys:

```json
{
  "value0": {
    "T_imu_cam": [ ... ],
    "intrinsics": {
      "camera_type": "<string>",
      "intrinsics": { ... }
    },
    "resolution": [width, height]
  }
}
```

`"value0"` is the top-level key introduced by `cereal`'s `JSONOutputArchive` when serializing the `Calibration` struct (`thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp`).

### 3.2 Camera Intrinsics Object

The `"intrinsics"` object inside `"value0"` contains two keys:

| Key | Type | Description |
|-----|------|-------------|
| `"camera_type"` | `string` | One of the eight strings listed in Section 2.10 |
| `"intrinsics"` | `object` | Model-specific parameter key-value pairs (see Section 2) |

### 3.3 Minimal Example: Pinhole

```json
{
  "value0": {
    "T_imu_cam": {
      "px": 0.0, "py": 0.0, "pz": 0.0,
      "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
    },
    "intrinsics": {
      "camera_type": "pinhole",
      "intrinsics": {
        "fx": 554.256,
        "fy": 554.256,
        "cx": 480.0,
        "cy": 360.0
      }
    },
    "resolution": [960, 720]
  }
}
```

### 3.4 Minimal Example: Double Sphere

```json
{
  "value0": {
    "intrinsics": {
      "camera_type": "ds",
      "intrinsics": {
        "fx": 285.0,
        "fy": 285.0,
        "cx": 320.0,
        "cy": 240.0,
        "xi": -0.27,
        "alpha": 0.58
      }
    },
    "resolution": [640, 480]
  }
}
```

---

## 4. Calibration Ingestion Pipeline

### 4.1 Overview

The diagram below shows the complete path from the ROS 2 launch file to the point where calibration data is consumed inside `TrackMonocular()`.

```
ROS 2 Launch File (vio_sim.launch.py)
        │
        │  calibration_file_path
        │  configuration_file_path
        ▼
BasaltSLAMNode  (src/node.cpp)
        │  passes paths to BasaltSLAM constructor
        ▼
BasaltSLAM       (src/slam.cpp)
        │  calls Controller::initialize()
        ▼
Controller::load_config()   (src/controller.cpp, include/basalt/controller.h)
        │
        │  std::ifstream(calib_path_)
        ▼
cereal::JSONInputArchive
        │
        │  archive(calib_)
        ▼
headers_serialization.h  (thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h)
        │  deserializes camera_type + per-model intrinsics
        ▼
GenericCamera<Scalar>::visitAllTypes()  (thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp)
        │  matches camera_type string → constructs camera object
        ▼
Calibration object  (calib_)  (thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp)
        │  stored as member of Controller
        ▼
TrackMonocular()
        │  uses calib_.intrinsics[cam_id] for projection / unprojection
        ▼
VIO tracking pipeline
```

### 4.2 ROS 2 Node - Parameter Declaration

In `src/node.cpp`, the `BasaltSLAMNode` constructor declares two filesystem parameters:

```cpp
this->declare_parameter<std::string>("calibration_file_path", "");
this->declare_parameter<std::string>("configuration_file_path", "");
```

These values are populated by the launch file `vio_sim.launch.py`:

```python
Node(
    package='basalt_slam',
    executable='basalt_slam_node',
    parameters=[{
        'calibration_file_path': '/path/to/tello_calib.json',
        'configuration_file_path': '/path/to/basalt_config.json',
    }]
)
```

If either parameter is left as an empty string, `TrackMonocular()` is invoked with empty paths and the `Controller` (`include/basalt/controller.h`) will fail to load the calibration, resulting in a crash at start-up. This is the root cause of the early `TrackMonocular()` crash documented in the simulation integration notes.

### 4.3 Controller - File Loading

Inside `Controller::load_config()` (`src/controller.cpp`, `include/basalt/controller.h`):

```cpp
std::ifstream calib_stream(calib_path_);
cereal::JSONInputArchive archive(calib_stream);
archive(calib_);
```

`calib_` is of type `basalt::Calibration<double>`, declared in:

```
thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp
```

The `cereal` archive drives deserialization by calling the `serialize` overloads registered in:

```
thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h
```

### 4.4 Serialization - `headers_serialization.h`

The `headers_serialization.h` header (`thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h`) registers `cereal` serialization methods for all `basalt` data structures. For the camera intrinsics this involves two levels:

Level 1, `Calibration` struct (`thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp`): the archive reads the `"value0"` top-level key and dispatches to the per-camera intrinsics serializer.

Level 2, `GenericCamera<Scalar>` variant serializer (`thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp`): the camera type string is read from `"camera_type"`. The matching camera object is then constructed and its model-specific parameters (e.g. `fx`, `fy`, `xi`, `alpha`) are read from the `"intrinsics"` sub-object.

### 4.5 Camera Model Selection - `visitAllTypes()`

The `"camera_type"` string dispatches to the correct C++ camera class via compile-time recursion over the `VariantT` variant type inside `GenericCamera` (`thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp`). The relevant code from `generic_camera.hpp` is:

```cpp
template <int I>
static void visitAllTypes(GenericCamera<Scalar>& res,
                          const std::string& name) {
  if constexpr (I >= 0) {
    using cam_t = typename std::variant_alternative<I, VariantT>::type;
    if (cam_t::getName() == name) {
      cam_t val;
      res.variant = val;
    }
    visitAllTypes<I - 1>(res, name);
  }
}
```

At runtime, this iterates backwards over all variant alternatives and instantiates the first type whose static `getName()` matches the JSON string. For example:

```
camera_type = "pinhole"
  → PinholeCamera::getName()       == "pinhole"  ✓  → construct PinholeCamera
  → DoubleSphereCamera::getName()  == "ds"       ✗
  → ...
```

```
camera_type = "ds"
  → PinholeCamera::getName()       == "pinhole"  ✗
  → DoubleSphereCamera::getName()  == "ds"       ✓  → construct DoubleSphereCamera
  → ...
```

Once constructed, the camera object is stored inside `calib_.intrinsics[cam_id]` and is available to the VIO estimator for all subsequent projection and unprojection operations.

### 4.6 Consumption in `TrackMonocular()`

`TrackMonocular()` receives a pointer to the populated `Calibration` object (`thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp`). For each incoming image frame, it retrieves the intrinsics for camera index 0:

```cpp
const auto& cam = calib_.intrinsics[0];
```

All feature tracking, landmark projection, and Jacobian computation downstream of this call uses the virtual dispatch interface of `GenericCamera<Scalar>` (`thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp`), meaning the rest of the pipeline is fully agnostic to which concrete camera model was selected.

---

## 5. Tello Gazebo Camera Configuration

### 5.1 Source of Intrinsics

The Tello drone's simulated camera is defined in:

```
tello_description/urdf/tello.xml
```

The ROS 2 Gazebo camera plugin (`libgazebo_ros_camera.so`) reads the sensor parameters from this file and computes the camera intrinsics on start-up. The relevant URDF parameters are:

| URDF Parameter | Value |
|---|---|
| Horizontal FOV | `0.96` rad (~55°) |
| Image width | `960` px |
| Image height | `720` px |
| Distortion coefficients | `0.0, 0.0, 0.0, 0.0, 0.0` |

### 5.2 Derived Intrinsics

From the horizontal FOV and image resolution, `libgazebo_ros_camera.so` derives the focal lengths at run time:

$$f_x = f_y = \frac{W}{2 \tan(\text{HFOV}/2)} = \frac{960}{2 \tan(0.48)} \approx 554.26 \text{ px}$$

The principal point is placed at the image centre:

$$c_x = W / 2 = 480.0 \text{ px}, \qquad c_y = H / 2 = 360.0 \text{ px}$$

All five distortion coefficients are set to zero. These values are published on the `/camera_info` topic and can be confirmed at runtime with:

```bash
ros2 topic echo /camera_info
```

### 5.3 The Gazebo Camera Plugin

`libgazebo_ros_camera.so` is Gazebo's standard ROS 2 camera sensor plugin. It is attached to a `<sensor type="camera">` block inside the robot's SDF/URDF description (here, `tello_description/urdf/tello.xml`) and is responsible for rendering the simulated view, computing the ideal camera intrinsics from the declared field of view and resolution, applying (optional) distortion, and publishing the result on `/image_raw` and `/camera_info`.

Internally, Gazebo's camera sensor implements a standard pinhole projection model, augmented with an optional plumb-bob (radial-tangential) distortion stage equivalent in form to OpenCV's model, the same family described for `pinhole-radtan8` in Section 2.9. When the distortion coefficients are all zero, as is the case for the Tello simulation, this reduces exactly to the ideal linear pinhole model of Section 2.2, which is why `"pinhole"` is the correct `camera_type` for this simulation (see Section 5.1 and Section 6).

### 5.4 Relevant XML Parameters in `tello.xml`

The `<sensor type="camera">` block in `tello_description/urdf/tello.xml` contains a `<camera>` tag with the parameters below. Only the parameters relevant to the current calibration setup are described; a full Gazebo SDF camera reference is out of scope for this document.

| XML Tag | Meaning | Relevance to Calibration |
|---|---|---|
| `<horizontal_fov>` | Horizontal field of view of the simulated lens, in radians | Used with image width to derive $f_x$ (and $f_y$, since pixels are assumed square) in Section 5.2 |
| `<image><width>`, `<image><height>` | Rendered image resolution in pixels | Sets the `resolution` field of the calibration JSON and the denominator of the focal length derivation |
| `<image><format>` | Pixel encoding of the rendered image (e.g. `R8G8B8`) | Affects the ROS image message encoding, not the calibration geometry |
| `<clip><near>`, `<clip><far>` | Near and far clipping planes of the render camera | Rendering-only; does not affect the projection model or calibration values |
| `<distortion><k1>`, `<k2>`, `<k3>`, `<p1>`, `<p2>` | Plumb-bob radial (`k1`-`k3`) and tangential (`p1`, `p2`) distortion coefficients applied by the plugin | All zero in the current Tello URDF, which is why the pinhole model (no distortion) is appropriate rather than `pinhole-radtan8` |
| `<distortion><center>` | Normalised image-plane centre used as the origin for the distortion function | Only meaningful when the distortion coefficients above are non-zero |
| `<noise>` (type, mean, stddev) | Pixel-level Gaussian noise added to the rendered image | Affects measurement noise realism, not the intrinsic geometry itself |

Because every distortion coefficient in the current Tello URDF is zero, the plugin's internal plumb-bob stage is a no-op, and the published `/camera_info` intrinsics reduce to the closed-form pinhole values derived in Section 5.2.

### 5.5 Implication for the Calibration JSON

Because the Gazebo camera is an ideal undistorted pinhole camera, the calibration JSON for the Tello simulation must use `"camera_type": "pinhole"` with the four parameters derived above.

---

## 6. Discussion: Choice of Camera Model for the Tello Simulation

### 6.1 Current State

The sample calibration JSON bundled with the SLAM stack uses `"camera_type": "ds"` (Double Sphere), with `xi` and `alpha` values inherited from a physical wide-angle camera calibration. This is inconsistent with the Gazebo simulation, which produces images from an ideal, zero-distortion pinhole camera (Section 5).

### 6.2 Consequence

Using the DS model for a pinhole camera introduces a model mismatch, in the sense introduced in Section 2.1.3: the sphere-based distortion family that the Double Sphere model is designed to represent has no counterpart in the actual imaging geometry produced by Gazebo. The projection function used during feature tracking and landmark triangulation will therefore differ from the true imaging geometry of the simulated sensor. In practice this results in small but systematic reprojection errors that grow with distance from the principal point, degrading the accuracy of pose estimates and landmark positions.

### 6.3 Recommended Configuration

For the Tello Gazebo simulation, the calibration JSON should specify:

```json
{
  "value0": {
    "T_imu_cam": {
      "px": 0.0, "py": 0.0, "pz": 0.0,
      "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
    },
    "intrinsics": {
      "camera_type": "pinhole",
      "intrinsics": {
        "fx": 554.256,
        "fy": 554.256,
        "cx": 480.0,
        "cy": 360.0
      }
    },
    "resolution": [960, 720]
  }
}
```

When the supervisor provides physical drone calibration data, the `camera_type` and intrinsic keys should be updated to match the lens model determined by the calibration tool used (e.g. Kalibr). At that point, transitioning from `"pinhole"` to `"ds"` or `"kb4"` may be appropriate if the physical lens exhibits significant fisheye distortion.

### 6.4 The `tello_calib.json` File

The configuration in Section 6.3 has been written out as a standalone calibration file, `tello_calib.json`, and placed under the project's data directory at:

```
data/tello_calib.json
```

This file should be referenced by the `calibration_file_path` parameter in `vio_sim.launch.py` (Section 4.2) in place of the existing Double Sphere calibration, so that the Tello Gazebo simulation loads a `"pinhole"`-type calibration consistent with the Gazebo camera plugin's actual (distortion-free) imaging geometry.

### 6.5 Verifying the New Calibration

After pointing `calibration_file_path` at `data/tello_calib.json`, the following checks confirm that the simulation is using the intended calibration:

1. Confirm `Controller::load_config()` (`src/controller.cpp`) parses the file without throwing, and that `TrackMonocular()` no longer crashes at start-up due to empty calibration paths (Section 4.2).
2. Echo `/camera_info` during simulation (Section 5.2) and confirm the published `fx`, `fy`, `cx`, `cy` values match `data/tello_calib.json`.
3. Run the VIO pipeline against the simulation and confirm `GrabImage()` continues receiving frames at the expected rate, with feature tracks and reprojection residuals behaving consistently rather than showing the systematic, radius-dependent error growth described in Section 6.2.

### 6.6 Extensibility

The `visitAllTypes()` mechanism (Section 4.5) means that adding support for a new camera model requires only:

1. Implementing the projection, unprojection, and Jacobian methods in a new header under `thirdparty/basalt-headers/include/basalt/camera/`.
2. Registering the new type in the `VariantT` alias inside `thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp`.
3. Adding a `cereal` serialization specialisation in `thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h`.

No changes are needed to the ingestion pipeline, the `Controller`, or `TrackMonocular()`.

---

## 7. Key Classes and Interfaces

### 7.1 `Calibration<Scalar>`

- Header path: `thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp`
- Source path: N/A (template struct defined in header)
- Purpose: Top-level calibration container deserialized from the calibration JSON. Holds the camera-IMU extrinsics, the per-camera intrinsics (as `GenericCamera<Scalar>`), and the image resolution. Populated once at start-up and consumed for the lifetime of the VIO pipeline.
- Inheritance: None.
- Variables:
  - `T_imu_cam`: rigid-body extrinsic transform(s) between the IMU and each camera.
  - `intrinsics`: array of `GenericCamera<Scalar>` objects, one per calibrated camera.
  - `resolution`: image width and height in pixels.

### 7.2 `GenericCamera<Scalar>`

- Header path: `thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp`
- Source path: N/A (template struct defined in header)
- Purpose: A variant wrapper (`VariantT`) over all eight concrete camera model types. Provides a uniform interface for projection, unprojection, and Jacobian computation regardless of which concrete model was selected at run time.
- Inheritance: None (composition over `std::variant`).
- Key methods:
  - `static void visitAllTypes<I>(GenericCamera<Scalar>& res, const std::string& name)`: compile-time recursive dispatch that matches the `camera_type` string against each variant alternative's `getName()` and constructs the matching concrete camera type (Section 4.5).

### 7.3 Concrete Camera Model Classes

Each of the following implements the projection function $\pi$, unprojection function $\pi^{-1}$, and their Jacobians described in Section 2.1.1, and exposes a static `getName()` used by `visitAllTypes()` (Section 7.2).

| Class | Header Path | `getName()` |
|---|---|---|
| `PinholeCamera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/pinhole_camera.hpp` | `"pinhole"` |
| `DoubleSphereCamera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/double_sphere_camera.hpp` | `"ds"` |
| `ExtendedUnifiedCamera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/extended_unified_camera.hpp` | `"eucm"` |
| `UnifiedCamera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/unified_camera.hpp` | `"ucm"` |
| `KannalaBrandt4Camera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/kannala_brandt4_camera.hpp` | `"kb4"` |
| `FovCamera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/fov_camera.hpp` | `"fov"` |
| `BalCamera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/bal_camera.hpp` | `"bal"` |
| `PinholeRadtan8Camera<Scalar>` | `thirdparty/basalt-headers/include/basalt/camera/pinhole_radtan8_camera.hpp` | `"pinhole-radtan8"` |

### 7.4 `headers_serialization.h`

- Header path: `thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h`
- Source path: N/A (serialization free functions defined in header)
- Purpose: Registers the `cereal` `serialize` overloads for every `basalt` data structure involved in calibration, including `Calibration<Scalar>` and each concrete camera model. Drives the two-level deserialization described in Section 4.4.
- Inheritance: N/A (free functions, not a class).
- Key functions: `serialize(Archive&, Calibration<Scalar>&)`, and one `serialize` overload per concrete camera model listed in Section 7.3.

### 7.5 `Controller`

- Header path: `include/basalt/controller.h`
- Source path: `src/controller.cpp`
- Purpose: Owns the calibration and configuration file paths, performs deserialization of the calibration JSON into a `Calibration<double>` instance, and exposes the populated calibration to the rest of the VIO pipeline.
- Inheritance: None documented in this scope.
- Variables:
  - `calib_path_`: filesystem path to the calibration JSON, populated from the `calibration_file_path` ROS 2 parameter.
  - `calib_`: the deserialized `basalt::Calibration<double>` instance.
- Key methods:
  - `void load_config()`: opens `calib_path_`, drives a `cereal::JSONInputArchive` over `calib_`, and populates the calibration object (Section 4.3).

### 7.6 `BasaltSLAMNode`

- Header path: N/A (declared and defined in `src/node.cpp`)
- Source path: `src/node.cpp`
- Purpose: The ROS 2 node entry point. Declares the `calibration_file_path` and `configuration_file_path` parameters and passes them through to `BasaltSLAM`.
- Inheritance: ROS 2 `rclcpp::Node`.
- Variables: `calibration_file_path`, `configuration_file_path` (declared ROS 2 parameters, Section 4.2).

### 7.7 `BasaltSLAM`

- Header path: N/A (declared and defined in `src/slam.cpp`)
- Source path: `src/slam.cpp`
- Purpose: Intermediate layer between the ROS 2 node and the `Controller`. Receives the calibration and configuration paths from `BasaltSLAMNode` and invokes `Controller::initialize()`.
- Inheritance: None documented in this scope.
- Key methods: constructor accepting calibration/configuration file paths; invokes `Controller::initialize()`.

### 7.8 `TrackMonocular()`

- Header path: N/A within scope of this document
- Source path: N/A within scope of this document
- Purpose: The per-frame entry point of the VIO tracking pipeline. Retrieves the camera intrinsics for camera index 0 from the populated `Calibration` object and uses them, via `GenericCamera<Scalar>`'s virtual dispatch, for feature tracking, landmark projection, and Jacobian computation (Section 4.6).
- Inheritance: N/A (free function / method, not further specified in this scope).

### 7.9 `libgazebo_ros_camera.so`

- Header path: N/A (Gazebo plugin, not part of the `basalt` or `Modular-Autonomous-Robotic-Systems` source tree)
- Source path: N/A (third-party Gazebo plugin binary)
- Purpose: Gazebo's ROS 2 camera sensor plugin. Reads the `<camera>` block of `tello_description/urdf/tello.xml`, renders the simulated view, applies the configured plumb-bob distortion (a no-op for the Tello configuration, Section 5.4), and publishes `/image_raw` and `/camera_info`.
- Inheritance: N/A (external Gazebo plugin).
- Relevant configuration: `<horizontal_fov>`, `<image><width>`/`<height>`, `<distortion>` coefficients (Section 5.4).

---

## 8. References

1. Usenko, V., Demmel, N., & Cremers, D. (2018). The Double Sphere Camera Model. International Conference on 3D Vision (3DV).
2. Khomutenko, B., Garcia, G., & Martinet, P. (2015). An Enhanced Unified Camera Model. IEEE Robotics and Automation Letters.
3. Geyer, C., & Daniilidis, K. (2000). A Unifying Theory for Central Panoramic Systems and Practical Implications. ECCV.
4. Kannala, J., & Brandt, S. S. (2006). A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses. IEEE TPAMI.
5. Devernay, F., & Faugeras, O. (2001). Straight Lines Have to Be Straight. Machine Vision and Applications.
6. Agarwal, S., Snavely, N., Seitz, S. M., & Szeliski, R. (2010). Bundle Adjustment in the Large. ECCV.
7. `thirdparty/basalt-headers/include/basalt/camera/` - Camera model headers.
8. `thirdparty/basalt-headers/include/basalt/camera/generic_camera.hpp` - `GenericCamera`, `visitAllTypes()`.
9. `thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h` - `cereal` serialization specialisations.
10. `thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp` - `Calibration` struct.
11. `src/controller.cpp`, `include/basalt/controller.h` - `Controller::load_config()`.
12. `src/node.cpp` - `BasaltSLAMNode` parameter declaration.
13. `tello_description/urdf/tello.xml` - Tello Gazebo camera URDF.
14. `data/tello_calib.json` - Pinhole calibration file for the Tello Gazebo simulation (Section 6.4).
15. `doc/VIO.md` - VIO pipeline overview.
16. `doc/LocalMapper.md` - Local mapper design.
17. `doc/Marginalisation.md` - Linearisation and marginalisation reference; source for the Key Classes and Interfaces section structure used in Section 7.
