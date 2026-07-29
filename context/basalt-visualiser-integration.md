# Driving `basalt::SlamVisualiser` from the ROS 2 node

**Date:** 2026-07-28. **Status:** implemented, container-side validation outstanding. **Files:** `include/slam/basalt/slam.h`, `src/basalt/slam.cpp`, `src/basalt/node.cpp`, `launch/basalt_slam.launch.py`, `../controllers/launch/basalt_slam_test.launch.py`, `ext/basalt/src/visualisation/visualiser.cpp`.
**Build-side companion:** [build-and-abi.md](build-and-abi.md) covers the `basalt_visualisation` target that makes this class linkable at all.

## What the visualiser is

`basalt::SlamVisualiser` opens one Pangolin window rendering the live VIO trajectory, the image grid, sliding-window landmarks, keyframes, a state plotter and the local map. It owns every Pangolin and OpenGL object in the project, so that the `basalt` library itself stays Pangolin-free. It is consumed twice, by `ext/basalt`'s standalone `basalt_slam` executable and by this package's `BasaltSLAMNode`.

## The three-call contract

`Start()`, `Run()` and `Stop()` are not independent. Reading `ext/basalt/src/visualisation/visualiser.cpp` establishes the following, and all three points are load-bearing.

- `Start()` at `:34` calls `pangolin::CreateWindowAndBind` at `:48` and then `glEnable(GL_DEPTH_TEST)`. A GL context is current on the thread that creates it, so **`Start()` and `Run()` must execute on the same thread.** `basalt_slam.cpp:232-239` states this in a comment and calls both from its main thread.
- `Start()` is also the wiring step. It sets `mpVio->out_vis_queue`, `out_state_queue`, the local mapper's `out_vis_queue` and the ground-truth queue at `:54-57`. Until it returns, the estimators have no taps and any frame processed in the meantime produces no visualisation data.
- `Stop()` at `:68` sets `mpQuitVisualiser` under `mpMtxQuitVisualiser`, then pushes nullptr sentinels and joins the three consumer threads. `Run()`'s loop reads that flag at `:240-243` and breaks, then tears down the GL objects and calls `pangolin::Quit()`. So `Stop()` followed by joining the render thread terminates cleanly, and `Stop()` is idempotent through `mpRunning.exchange(false)` at `:74`.

## How `BasaltSLAM` satisfies it

The node cannot give the visualiser its main thread, because that thread belongs to the ROS executor. `BasaltSLAM::InitialiseSlam` therefore runs the visualiser's entire GL lifetime on `mpVisualiserThread` and hands the startup result back with a `std::promise`.

```cpp
std::promise<void> startedPromise;
std::future<void> started = startedPromise.get_future();
mpVisualiserThread = std::thread(&BasaltSLAM::RunSlamVisualiser, this,
                                 std::move(startedPromise));
started.get();
```

```cpp
void BasaltSLAM::RunSlamVisualiser(std::promise<void> startedPromise) {
    try {
        mpSlamVisualiser->Start();
    } catch (...) {
        startedPromise.set_exception(std::current_exception());
        return;
    }
    startedPromise.set_value();
    mpSlamVisualiser->Run();
}
```

Three properties fall out of this shape, and each answers a defect that the earlier arrangement had.

- **Window creation, rendering and teardown all happen on `mpVisualiserThread`.** The earlier code called `Start()` inline on the ROS thread and only `Run()` on the new thread, which bound the context to one thread and issued every subsequent GL call from another.
- **`InitialiseSlam` does not return until `Start()` has returned**, so the estimator taps exist before `on_activate` creates the image subscription and frames begin to flow.
- **`get()` rather than `wait()`** rethrows a `Start()` failure on the caller's thread. The common failure is no reachable `DISPLAY`. Letting the exception escape the thread body instead would call `std::terminate`, and returning without fulfilling the promise would block the waiter forever.

`StopSlamVisualiser()` returns early when `mpSlamVisualiser` is null. This matters because `use_visualisation` defaults to false, so no visualiser object exists on the default path, while `BasaltSLAMNode::on_deactivate` calls `mpSlam->StopSlamVisualiser()` unconditionally. Without the guard, every deactivation on the default configuration dereferences a null `unique_ptr`.

## The parameter and launch-argument path

The flag travels through four names, and they are not spelled alike at every hop.

| Layer | Name | Notes |
|---|---|---|
| `basalt_slam_test.launch.py` argument | `use-visualisation` | Hyphenated, declared with `choices=["True","False"]`, forwarded into the include |
| `basalt_slam.launch.py` argument | `use-visualisation` | Hyphenated, same choices, default `False` |
| Node parameter | `use_visualisation` | Underscored, declared `declare_parameter<bool>` at `src/basalt/node.cpp:43` |
| C++ | `mpUseVisualisation` | Read in `on_configure` at `:62`, passed to the `BasaltSLAM` constructor at `:102-104` |

The launch file wraps the substitution in `ParameterValue(use_visualisation, value_type=bool)`. This is required rather than cosmetic. A `LaunchConfiguration` resolves to text, and `declare_parameter<bool>` rejects a string override with `InvalidParameterTypeException`, so the target type has to be stated instead of inferred. Note that `use_sim_time` in the same file still passes a bare `LaunchConfiguration`, which is a pre-existing inconsistency and a candidate defect worth checking the next time that path is exercised.

The same value also reaches `basalt::Controller::initialize` as its `enableVisualisation` argument at `src/basalt/slam.cpp:47-54`. `SlamVisualiser::Start()` gates on `mpController.IsVisualisationEnabled()` at `:35` and returns immediately when it is false, so the two must agree. They do, because both read the one `useVisualisation` argument.

## Gotchas worth remembering

- **A Pangolin window is thread-affine, not process-global.** Any future refactor that moves `Start()` away from the thread that runs `Run()` reintroduces the same silent failure, where GL calls execute with no current context rather than raising anything legible.
- **`use_visualisation=True` needs a reachable X display.** Under `djinn` the container must have `DISPLAY` and an X socket mounted. Headless runs must leave it false, which is the default at every layer.
- **The gate is duplicated by design.** `Controller`'s `enableVisualisation` decides whether the estimators publish visualisation data at all, and `SlamVisualiser::Start()` re-checks it. Setting one without the other yields a window with no data, or data with no window.
