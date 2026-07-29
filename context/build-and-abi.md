# Building `slam`: `SLAM_TYPE` branches, the `ext/basalt` scope boundary, and the Eigen ABI pin

**Date:** 2026-07-25. **Status:** the pin, both guards and the per-architecture `CXX_MARCH` selection are **implemented**; the clean-rebuild verification is still outstanding. **Files:** `CMakeLists.txt`, `ext/basalt/CMakeLists.txt`, `/ws/scripts/build_ros_packages.sh`.
**Full derivation of the ABI problem:** [`SIMD.md`](SIMD.md). **Investigation record:** [`/ws/ros_ws/context/simd-eigen-abi-mismatch.md`](../../../context/simd-eigen-abi-mismatch.md).

## Configuration structure

`CMakeLists.txt:4-5` declares a cache variable that selects the SLAM back-end, and the whole file is one large two-way branch:

```cmake
set(SLAM_TYPE "BASALT" CACHE STRING "Type of SLAM system to build (ORBSLAM3 or BASALT)")
set_property(CACHE SLAM_TYPE PROPERTY STRINGS "ORBSLAM3" "BASALT")
```

| Lines | Branch | Builds |
|---|---|---|
| 95-160 | `if(SLAM_TYPE STREQUAL "ORBSLAM3")` | `orbslam3_mono_node`, `orb_slam3_lib`, `DBoW2`, `g2o` |
| 161-215 | `elseif(SLAM_TYPE STREQUAL "BASALT")` | `add_compile_definitions(...)` (:182), `add_subdirectory(ext/basalt)` (:185), then `basalt_slam_node` (:187-193) from `src/{node,slam}.cpp` and `src/basalt/{slam,node,driver}.cpp` |

**The default is `BASALT`**, so a plain `colcon build` never configures the ORB-SLAM3 branch. That is why defects confined to lines 95-157 (see "Install-rule defects" below) went unnoticed.

## The scope boundary that caused the ABI split

`add_subdirectory(ext/basalt)` at `:185` creates a **new CMake directory scope**. Inside it, `ext/basalt/CMakeLists.txt` does:

```cmake
if(NOT CXX_MARCH)
  set(CXX_MARCH native)                                                     # :85-87
endif()

IF(NOT APPLE OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(BASALT_MARCH_FLAGS "-march=${CXX_MARCH}")                             # :89-90
ENDIF()

set(CMAKE_CXX_FLAGS "${BASALT_CXX_FLAGS} ${BASALT_MARCH_FLAGS} ${BASALT_PASSED_CXX_FLAGS}")   # :265
```

`set()` without `PARENT_SCOPE` is directory-scoped, so `:265` changes `CMAKE_CXX_FLAGS` for `ext/basalt/**` **only**. `basalt_slam_node`, declared in the parent directory at `:187`, never receives `-march`. A warning comment now sits above `if(NOT CXX_MARCH)` recording that the flag is an ABI switch and does not cross the scope boundary. The generated build files show it plainly:

```
# build/slam/ext/basalt/CMakeFiles/basalt.dir/flags.make
CXX_FLAGS = … -DEIGEN_DONT_PARALLELIZE -march=native -Wall -O3 -std=c++17 -O3 -g …

# build/slam/CMakeFiles/basalt_slam_node.dir/flags.make
CXX_FLAGS =  -Wall -O3 -std=c++17 -O2 -g -DNDEBUG          ← no -march
```

`-march=native` on an AVX-512 host defines `__AVX512F/DQ/BW/VL/CD__`, Eigen reads those macros and raises `EIGEN_MAX_ALIGN_BYTES` from 16 to 64, and every fixed-size vectorizable Eigen type changes size and alignment. `basalt::Controller` became 1056 bytes/align 32 in the library and 1008 bytes/align 16 in the node — a 48-byte heap overflow on construction, surfacing as a `SIGSEGV` inside `ld.so` during an unrelated `dlopen`.

## The contract: `EIGEN_MAX_ALIGN_BYTES` and `NRT_EIGEN_ABI_PIN`

The fix, implemented 2026-07-25, is a directory-scoped compile definition placed **before** `add_subdirectory(ext/basalt)`:

```cmake
add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=16 NRT_EIGEN_ABI_PIN=16)
```

Why this works, and why the mechanism matters:

- `add_compile_definitions()` populates the **`COMPILE_DEFINITIONS` directory property**, inherited by every subdirectory added *after* the call and every target created in this directory *after* the call. One statement at `:182` therefore covers `basalt`, `basalt_slam_node`, and basalt's own executables. Ordering is the whole mechanism: move it below `add_subdirectory` and it silently does nothing.
- Directory compile definitions land in **`CXX_DEFINES`**, not `CXX_FLAGS`. This is load-bearing: `ext/basalt/CMakeLists.txt:265` *overwrites* `CMAKE_CXX_FLAGS` wholesale, so anything routed through `CXX_FLAGS` would be silently discarded. `CXX_DEFINES` survives.
- **16 rather than 32** so that basalt agrees with every distribution-built ROS 2 package in the same process (`rclcpp`, `tf2_eigen`, `tf2_geometry_msgs`, `cv_bridge`), all compiled without `-march` and therefore at 16. Choosing 32 would fix the internal boundary and leave the same latent hazard against the rest of the process.
- `NRT_EIGEN_ABI_PIN` carries the same value to `static_assert` guards on **both** sides of the boundary — `include/slam/slam.hpp` (parent, placed right after `#include <Eigen/Geometry>`) and `ext/basalt/src/controller.cpp` (library, after the includes and before `namespace basalt`). Two guards are needed because the two sides share no header: `ext/basalt` never includes anything from `include/slam/`. Both are wrapped in `#ifdef NRT_EIGEN_ABI_PIN` so that a standalone upstream basalt build (which its own `.gitlab-ci.yml` performs) is unaffected. `controller.cpp` is the right library-side home because it is the definition site of the exact type whose layout diverged.
- **What the assert actually catches.** It compares Eigen's `EIGEN_MAX_ALIGN_BYTES` against the pin, so it passes trivially while both definitions reach the TU and fires the moment the `-D` stops arriving (a reordered `add_compile_definitions`, a target created outside the directory scope, a subproject re-deriving the value from `-march`).

**If you change the pin, change it in all three places** — the `add_compile_definitions` call and both `static_assert`s pick it up automatically from `NRT_EIGEN_ABI_PIN`, so in practice only the one CMake line moves. That is the point of routing the value through a second macro rather than hardcoding `16` in the asserts.

## `CXX_MARCH` selection

`/ws/scripts/build_ros_packages.sh` historically passed no `-DCXX_MARCH`, so basalt's `native` default applied. That is wrong here for three reasons: `install/` is a mounted tree that outlives the container and can be carried to a host without AVX-512 (`SIGILL`); builds are not reproducible between machines; and `native` is meaningless under `docker buildx --platform`, where on aarch64 it silently degrades to the generic baseline rather than erroring.

It now selects per architecture at `:26-34` and passes `-DCXX_MARCH=${CXX_MARCH}` through `--cmake-args` at `:37`. Per architecture is required because `ext/basalt/CMakeLists.txt:89` still emits `-march=` on Linux/aarch64 (its guard only suppresses the flag on Apple Silicon), where an x86 value is a hard compiler error:

| `uname -m` | `CXX_MARCH` | Rationale |
|---|---|---|
| `x86_64` | `x86-64-v3` | AVX2 + FMA; Haswell (2013) / Zen 1 and later. Matches upstream basalt's `haswell` choice for its packaged `.deb` jobs. |
| `aarch64` | `armv8.2-a+fp16+dotprod` | Jetson Orin Nano, Cortex-A78AE. No SVE on this part. |

The block is guarded by `if [ -z "${CXX_MARCH}" ]`, so `CXX_MARCH=native ./scripts/build_ros_packages.sh` still reproduces the old behaviour exactly for a benchmark. `CXX_MARCH` is read **only** by `ext/basalt/CMakeLists.txt:85`; no other package in `ros_ws` consults it (verified by grep), so passing it globally through `colcon --cmake-args` is inert everywhere else. Expect floating-point results to differ in the last digits from a `native` build, because FMA contraction and vector width differ between arch levels — **do not validate by diffing trajectory files**, compare against ground truth with an ATE/RPE metric. Upstream basalt agrees `native` is dev-only: `ext/basalt/.gitlab-ci.yml:6` uses `native` for the dev build but pins `haswell` for every packaged job (`:158,:170,:183`).

## Install-rule defect in the ORBSLAM3 branch (fixed 2026-07-25)

`CMakeLists.txt:143-145` used to contain:

```cmake
  install(TARGETS
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
```

The target list was empty, and `PYTHON_INSTALL_DIR` is never `set()` in this file, so the destination expanded to the absolute path `/slam`. **Deleted** — the Python side is installed by `ament_python_install_package(${PROJECT_NAME})` and the C++ targets by the `install(TARGETS orbslam3_mono_node orb_slam3_lib DBoW2 g2o …)` block below it. Inert under the default `SLAM_TYPE=BASALT`, but it blocked `-DSLAM_TYPE=ORBSLAM3`. A NOTE now sits in that branch recording that the `EIGEN_MAX_ALIGN_BYTES` pin must be added there too if `-march` is ever introduced into the ORB-SLAM3 build; ORB-SLAM3 and its vendored g2o/DBoW2 are currently built without it and are self-consistent at 16.

`-DSLAM_TYPE=ORBSLAM3` remains blocked for unrelated reasons — `slam/launch/orbslam3.launch.py` does not exist and `mono_driver_node.py` uses `MultiThreadedExecutor` without importing it. Out of scope of the ABI work.

## The Pangolin propagation boundary (fixed 2026-07-28)

The VIO visualiser work made `include/slam/basalt/slam.h:7` include `basalt/visualisation/visualiser.h`, which opens with `#include <pangolin/pangolin.h>`. That pulled Pangolin into all three of this package's basalt translation units, `src/basalt/{slam,node,driver}.cpp`, and the build broke with roughly two hundred errors of the form `'glGenBuffers' was not declared in this scope`, raised from inside `/usr/local/include/pangolin/gl/gl.hpp` and `glsl.h`.

The error signature is the diagnosis. Every undeclared name is an OpenGL 1.5 or later entry point, `glGenBuffers`, `glUseProgram`, `glBindFramebufferEXT`, `glCopyImageSubDataNV`. Every name the compiler did resolve, and offered as a spelling correction, is OpenGL 1.1, `glDrawBuffer`, `glReadBuffer`, `glSelectBuffer`. Mesa's `<GL/gl.h>` declares exactly the 1.1 set and nothing beyond it, so the translation unit had reached Pangolin's headers with no GLEW in sight. `ext/basalt/thirdparty/Pangolin/include/pangolin/gl/glplatform.h:58` includes `<GL/glew.h>` unconditionally, so any consumer of a Pangolin header must be given GLEW's include directory.

Two facts about the vendored tree explain why nothing supplied it.

- `basalt` is deliberately Pangolin-free. `add_library(basalt SHARED)` at `ext/basalt/CMakeLists.txt:306` does not compile `src/visualisation/visualiser.cpp` or `src/visualisation/utils.cpp`, and `target_link_libraries(basalt …)` at `:309` does not name `pangolin`. The design intent is recorded in the header comment of `visualisation/visualiser.h`. Before the fix below, the two visualiser translation units were compiled directly into each consuming executable, which is what `basalt_slam` did in its own `add_executable` source list.
- `basalt_slam_node` linked only `basalt`, so it inherited no Pangolin include directory, no GLEW include directory and no Pangolin compile definitions. `pangolin/pangolin.h` still resolved, but only because an unrelated Pangolin installation sits in `/usr/local/include`, a default system search path. The package was therefore compiling against a stale system Pangolin rather than the vendored one under `ext/basalt/thirdparty/Pangolin`.

## The `basalt_visualisation` target

The fix introduces one reusable target rather than repeating a source list per consumer. `ext/basalt/CMakeLists.txt:434-438` now declares

```cmake
add_library(basalt_visualisation STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/visualisation/visualiser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/visualisation/utils.cpp
)
target_link_libraries(basalt_visualisation PUBLIC basalt pangolin)
```

and both consumers link it, `basalt_slam` at `ext/basalt/CMakeLists.txt:470-471` and `basalt_slam_node` at `CMakeLists.txt:205-210`. Neither names `pangolin` any more, and neither lists the visualiser sources.

The `PUBLIC pangolin` link is the entire mechanism. `ext/basalt/thirdparty/Pangolin/src/CMakeLists.txt:176` appends `${GLEW_INCLUDE_DIR}` to `USER_INC`, and `:635` exports `USER_INC` with `target_include_directories(${LIBRARY_NAME} SYSTEM PUBLIC …)`, so anything linking `pangolin` receives GLEW's include directory as an `-isystem` flag together with the vendored Pangolin tree and its generated `config.h` directory. Explicit flags are searched before `/usr/local/include`, so the vendored Pangolin wins over the stale system one.

`STATIC` is deliberate. The two objects are small, every consumer compiled them directly before the target existed, so an archive reproduces exactly the same object placement, and no new shared object enters the install tree or the RPATH set. `CMAKE_POSITION_INDEPENDENT_CODE` is `ON` for that directory at `ext/basalt/CMakeLists.txt:100`, so the archive still links into shared consumers. `pangolin` itself is also static here, because `ext/basalt/thirdparty/CMakeLists.txt:11` sets `BUILD_SHARED_LIBS OFF` for the thirdparty scope, which is why no install rule was added for either.

One ABI consequence is worth recording. The visualiser translation units moved from the parent scope, where they received no `-march`, into `ext/basalt`'s scope, where `:265` supplies `-march=${CXX_MARCH}`. This is safe only because `add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=16 NRT_EIGEN_ABI_PIN=16)` at `CMakeLists.txt:183` precedes `add_subdirectory(ext/basalt)` and therefore reaches the new target too. The pin, not the arch flag, is what keeps `SlamVisualiser`'s `Eigen::aligned_vector<Eigen::Vector3d>` members laid out identically on both sides. Delete the pin and this target becomes a second ABI boundary of exactly the kind described above.

Two notes for whoever meets this next.

- **A header include is a build-system dependency.** Adding `#include <pangolin/…>` to a header that this package's targets consume obliges the target to link the library that owns it, even when no symbol is called. `unique_ptr` to an incomplete type would avoid the include entirely, since `~BasaltSLAM()` is already out of line, but `slam.h` currently declares the visualiser by value of its pointer with the full definition visible.
- **When two targets need the same third-party surface, export it from one target rather than repeating the recipe.** `basalt_visualisation` exists so that the next consumer of `visualiser.h` links one name and inherits Pangolin, GLEW and the visualiser objects together. The failure mode this prevents is a consumer that copies half the recipe, the sources without the link entry or the reverse, and rediscovers the GL 1.1 error wall.
- **A stale library on a default system path masks a missing link entry rather than causing a failure outright.** The build found headers it was never configured to find. If the errors from a vendored dependency name paths under `/usr/local`, the target is missing that dependency's link entry.

## Dead `find_package(Pangolin REQUIRED)` in the common section

`CMakeLists.txt:42` calls `find_package(Pangolin REQUIRED)` at the top level, above the `SLAM_TYPE` branch. No `Pangolin_INCLUDE_DIRS` or `Pangolin_LIBRARIES` variable is read anywhere in that file, and the ORBSLAM3 branch does not need it either, because `orb_slam3/CMakeLists.txt:6` issues its own `find_package(Pangolin REQUIRED)` and consumes the variables in its own scope at `:50,:57`. The call is therefore dead in both branches, while `REQUIRED` still makes a system Pangolin a hard configure-time requirement of the default BASALT build, which does not use one. Left in place as of 2026-07-28 because removing it touches the section shared with the ORBSLAM3 branch and the build failure did not depend on it. It is a safe deletion whenever that branch is next revisited.

It also proves something useful about target naming. Had `find_package(Pangolin)` created an imported target called `pangolin`, the vendored `add_library(pangolin …)` reached through `add_subdirectory(ext/basalt)` would have failed configuration on the duplicate name. Configuration succeeds, so `pangolin` refers unambiguously to the vendored target and the link entry described above cannot bind to the system copy.

## Gotchas worth remembering

- **`-march` is an ABI switch, not an optimisation flag, in any Eigen-based codebase.** Two translation units sharing a type must agree on it, exactly like `-std` or `_GLIBCXX_USE_CXX11_ABI`.
- **Any vendored subproject that assigns to `CMAKE_CXX_FLAGS` creates a silent ABI boundary with its parent's targets.** `add_subdirectory` scoping is the mechanism; nothing warns.
- **Never incrementally rebuild across an ABI change.** `rm -rf /ws/ros_ws/{build,install,log}/slam` before rebuilding, or `colcon` will mix old and new object files and reproduce the same class of bug with different numbers.
- **Keep `-g` in release builds.** `RelWithDebInfo` already does. The DWARF `DW_AT_byte_size` / `DW_AT_alignment` comparison is what made the diagnosis provable in minutes:
  ```bash
  objdump --dwarf=info <obj.o> | grep -A6 'DW_AT_name.*: Controller$' | grep -E "byte_size|alignment"
  ```
  Run it against one object from each side; if they differ, you have proven an ODR violation and there is nothing left to argue about.
- **This bug is x86-only.** AArch64/NEON is 128-bit, so `EIGEN_MAX_ALIGN_BYTES` is 16 on the Jetson Orin regardless of `-march`. It cannot reproduce on the target board — which is precisely why it survived.
- **`basalt_slam_node`'s own flags come from CMake's `RelWithDebInfo`** (`-O2 -g -DNDEBUG`), while basalt's come from its own overrides (`-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN`). The asymmetry is expected and harmless; only the `-march` difference matters.
