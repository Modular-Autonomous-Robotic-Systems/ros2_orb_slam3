# Basalt threading model and queue topology as driven by `BasaltSLAMNode`

**Date:** 2026-08-22. **Files:** `src/basalt/driver.cpp`, `src/basalt/node.cpp`, `src/basalt/slam.cpp`, `ext/basalt/src/controller.cpp`, `ext/basalt/src/vi_estimator/sqrt_keypoint_vio.cpp`, `ext/basalt/src/vi_estimator/local_mapper.cpp`, `ext/basalt/src/visualisation/visualiser.cpp`.

This records which thread executes which stage of the vendored basalt pipeline when it is driven from this package, and which queue edges block. It exists because the pipeline supports two execution models with entirely different threading, the node selects the less obvious one, and the resulting blocking edges have already produced two distinct deadlocks. Consult this before adding any queue to the pipeline or changing how callbacks are scheduled.

## Which execution model this package selects

`ext/basalt` supports a producer consumer model, where every stage owns a thread, and an event driven model, where stages are invoked synchronously on the caller's thread. The selector is the sixth argument to `Controller::initialize`.

`src/basalt/slam.cpp:48-55` passes `false`, selecting the event driven model. The consequences are that `FrameToFrameOpticalFlow` spawns no `processingLoop`, that `SqrtKeypointVioEstimator::initialize` takes the branch at `ext/basalt/src/vi_estimator/sqrt_keypoint_vio.cpp:201-205` which creates no `processing_thread` and registers no `proc_func`, and that `Controller::TrackMonocular` (`ext/basalt/src/controller.cpp:198-210`) runs optical flow and the complete visual inertial backend inline on its caller's thread.

Consequently the whole pipeline contains only two threads of consequence.

| Thread | Runs | Spawned at |
|---|---|---|
| ROS executor thread | image ingestion, optical flow, VIO backend, marginalisation | `src/basalt/driver.cpp:10` |
| local mapping thread | `LocalMapper::MapLocally`, BA, keyframe culling | `ext/basalt/src/vi_estimator/local_mapper.cpp:50`, unconditionally |

The mapping thread is spawned in `LocalMapper::Initialise` regardless of execution model, which is why it is the only genuine concurrency in the event driven configuration.

When `use_visualisation` is true, `SlamVisualiser::Start` (`ext/basalt/src/visualisation/visualiser.cpp:60-65`) adds three consumer threads and the Pangolin GL thread. The parameter defaults to false at `src/basalt/node.cpp:43`, so the default deployment has none of them.

## Executor and callback groups

`src/basalt/driver.cpp` uses `rclcpp::executors::MultiThreadedExecutor` as of 2026-08-23, and `BasaltSLAMNode` gives each subscription its own mutually exclusive callback group, created in the constructor and attached through `rclcpp::SubscriptionOptions` in `on_activate`. Groups must be created before `add_node`, because an executor collects a node's callback groups when the node is added and does not reliably observe groups created later. Corrected 2026-08-23. This paragraph previously recorded `rclcpp::spin` and a single default callback group as the shipped state, which is what made the inertial queue's blocking pop a self deadlock, described below.

Reentrant groups are not appropriate for either callback. `GrabImage` mutates `m_cvImPtr` and `mpCurrentFrame`, and `GrabIMU` mutates `mpLastIMUTimestamp`, so both require mutual exclusion against themselves.

## Blocking edges, and who sits on each side

Every inter-stage edge is a `tbb::concurrent_bounded_queue`. `pop` and `push` on that type both block without timeout or cancellation; `try_pop` and `try_push` are the non-blocking forms.

| Site | Operation | Capacity, set at | Other side of the edge |
|---|---|---|---|
| `sqrt_keypoint_vio.cpp:225,238` via `popFromImuDataQueue` | bounded `try_pop` loop, 5 s | 300, `sqrt_keypoint_vio.cpp:125` | produced by `GrabIMU`, its own executor thread since 2026-08-23 |
| `sqrt_keypoint_vio.cpp:308,323` via `popFromImuDataQueueNonBlocking` | `try_pop`, breaks on empty | 300 | non-blocking, correct |
| `controller.cpp:220` | `imu_data_queue.push` | 300 | consumed by the same executor thread |
| `sqrt_keypoint_vio.cpp:901` | `out_marg_queue.push` | 10, `controller.cpp:172` | consumed by the mapping thread; measured never full, size -1 before each push |
| `local_mapper.cpp:83` | `mpMargInputQueue.pop` | 10 | produced by the executor thread |
| `sqrt_keypoint_vio.cpp:621` | `out_vis_queue.try_push` | 20, `visualiser.cpp:39` | non-blocking since 2026-08-23, was a blocking `push` |
| `sqrt_keypoint_vio.cpp:594` | `out_state_queue.try_push` | 100, `visualiser.cpp:40` | non-blocking since 2026-08-23, was a blocking `push` |
| `local_mapper.cpp:252` | `out_vis_queue.try_push` | 4, `visualiser.cpp:41` | non-blocking, correct |

The pipeline holds exactly one mutex, `mpPosesToUpdateMutex`. It is taken at `sqrt_keypoint_vio.cpp:369-407` inside a closed block that ends before `optimize_and_marg` runs, and at `sqrt_keypoint_vio.h:215` for a map copy. There is no lock ordering cycle anywhere. Any hang in this pipeline is a queue problem, never a mutex problem, and searching for a lock cycle is wasted effort.

## Deadlock 1, the inertial self wait (steady state)

Observed as a hang after roughly seventy five keyframes when running against AirSim SITL, with the process alive and CPU at nil.

`popFromImuDataQueue` performs an unbounded blocking `imu_data_queue.pop`. Under the single threaded executor the only producer, `GrabIMU`, is dispatched by the thread that is blocked in the pop. The wait is a self deadlock by construction and nothing can release it.

The 300 sample buffer, about 1.8 seconds at the 167 Hz AP_DDS rate recorded in `ext/basalt/context/ap_dds_imu_stream.md`, conceals the defect until the buffer empties, and what empties it lives in the middleware rather than in basalt. The inertial subscription requests `rclcpp::SensorDataQoS()` at `src/basalt/node.cpp:127-129`, which is KEEP_LAST with a depth of five. While the executor is inside `GrabImage` it dispatches nothing, so arriving samples accumulate in that five deep history and everything older than the newest five is silently discarded by the middleware. At most five samples therefore enter `imu_data_queue` per image callback, while `ProcessFrame` consumes one sample per six milliseconds of image period. Admission per frame is about five plus the idle remainder of the image period divided by six milliseconds, consumption is the whole image period divided by six milliseconds, so the deficit per frame is the callback duration divided by six milliseconds less five. It is positive for any callback above thirty milliseconds. At the 7.42 Hz camera rate measured in `ext/basalt/context/airsim_camera_extrinsics.md`, an image period of about 135 ms, a fifty millisecond callback gives a deficit near three samples per frame and drains the three hundred sample buffer in roughly a hundred frames. Corrected 2026-08-23. This paragraph previously named the ten slot marginalisation queue and the blocking push at `sqrt_keypoint_vio.cpp:901` as the stall that empties the buffer. Instrumentation on the running stack reports that queue at size minus one immediately before each push, which for `tbb::concurrent_bounded_queue` denotes a consumer already parked in `pop`, so the mapper keeps pace, the queue never fills and that push never blocks.

The loop at `sqrt_keypoint_vio.cpp:297`, `while (imuData->t_ns <= curr_frame->t_ns)`, terminates only on a sample strictly newer than the current image stamp, so every frame requires at least one sample from beyond its own timestamp. There is no configuration in which the image stream may lead the inertial stream.

Diagnostic signature under `gdb -p <pid> -batch -ex "thread apply all bt"`: the executor thread inside `tbb::concurrent_bounded_queue::pop` beneath `popFromImuDataQueue` and `ProcessFrame`, with the mapping thread simultaneously idle in `pop` at `local_mapper.cpp:83`. An idle mapper alongside a blocked backend proves the marginalisation queue is empty, which distinguishes the deadlock from ordinary mapper latency.

Remediation is specified in [`../plans/basalt-local-mapper-deadlock.md`](../plans/basalt-local-mapper-deadlock.md).

## Deadlock 2, the shutdown sentinel (teardown)

Distinct failure, recorded fully in `ext/basalt/BUG.md`. `LocalMapper::Stop` sets `mpStopLocalMapping` and joins, but `MapLocally` evaluates that flag only at the top of each iteration and cannot observe it while blocked in `pop` at `local_mapper.cpp:83`. The sentinel that releases it is emitted only from the `proc_func` lambda that the event driven model never instantiates.

Both deadlocks share one origin. Blocking queue contracts written for the producer consumer model, where every edge separates two threads and backpressure is correct, were inherited unchanged by the event driven model, where the stages collapse onto one thread and those edges close into cycles. Before adding a queue to this pipeline, establish which thread sits on each side of it in both execution models.

## Latent null dereferences on the frame path

`ProcessFrame` can already return null at `sqrt_keypoint_vio.cpp:290`, `:300` and `:308`. Three call sites dereference without checking, and all three become materially more reachable once the inertial pop is given a timeout.

- `sqrt_keypoint_vio.cpp:224-230` dereferences the first popped sample unconditionally.
- `sqrt_keypoint_vio.cpp:236-251` breaks out of the initialisation loop on a null sample, then dereferences that same null pointer to derive the gravity aligned orientation via `FromTwoVectors`.
- `controller.cpp:203-204` dereferences the returned state to fill `tcw`.

## Mapper cost defects that drive the trigger

These lengthen the mapper cycle, degrade the map and, through `mpVioPoseUpdateCallback`, add to the per frame work of the thread that runs the image path. Corrected 2026-08-23. They were previously described as load bearing for deadlock 1 on the reasoning that they drive the marginalisation queue to capacity. The queue is measured never to fill, so they shorten the time to failure without being its cause.

`SelectKeyframesToCull` (`local_mapper.cpp:651-663`) appends a candidate inside its inner loop without breaking, so a keyframe redundant against k others is appended k times. `CullRedundantKeyframes` then repeats the full rehost, `removeFrame`, factor pruning, bag of words removal and track builder cleanup for frames it has already erased. With `mpCullCovisibilityThresh` at 0.5 and a fifty frame map the list holds hundreds of entries where it should hold a handful.

`IngestMargData` clears `mpNewKeyframesForTracking` on entry (`local_mapper.cpp:296`) while the drain loop at `:108-112` calls it once per batched packet, so only the last call's keyframes survive. The batch is consumed with `vecData.back()`, hence in reverse arrival order, so the survivors belong to the oldest packet. Every other packet's keyframes enter `frame_poses` but are never tracked, matched or culled, and the map grows past `mpMaxLocalMapSize`, inflating the quadratic covisibility scan further. This is the accumulation defect anticipated in `ext/basalt/local_mapper_optimisation.md` section 4.2, whose prescribed remedy was never implemented.

## Contract violation on the visualiser taps

`ext/basalt/src/visualisation/visualiser.cpp:36-37` states that producers use `try_push` so a full queue drops the newest frame instead of blocking the estimator threads. `LocalMapper` honours this at `local_mapper.cpp:252`, and `SqrtKeypointVioEstimator` honours it at `sqrt_keypoint_vio.cpp:594` and `:621` as of 2026-08-23. Corrected 2026-08-23. Those two sites previously used blocking pushes against queues of capacity 100 and 20, so enabling `use_visualisation` added a third independent stall path whenever the Pangolin thread stuttered.

The shutdown sentinel pushes at `:192-194` and `:217-219` must stay blocking. A dropped sentinel reproduces deadlock 2.

## Subscription history depth is a data loss mechanism, not a buffer

`rclcpp::SensorDataQoS()` is BEST_EFFORT, VOLATILE and KEEP_LAST(5). The depth is required reading whenever a callback on the same executor thread runs long. A history of five at a 167 Hz stream is thirty milliseconds, so any callback exceeding that duration causes the middleware to discard inertial messages, unreported and unrecoverable because the reliability is BEST_EFFORT. This is the admission side of deadlock 1 and it is invisible from inside basalt, since `imu_data_queue` simply receives fewer pushes than the publisher sent.

Deepening the history is not the remedy. It lengthens the interval before the same failure and does nothing about the fact that the consumer is not running. The remedy is a dedicated callback group and a multi threaded executor, which drains the history continuously.

## The four inertial pop sites divide into two kinds

Recorded because the division is not apparent from the code, which uses one method for all four.

The two initialisation sites, `sqrt_keypoint_vio.cpp:225` for the first sample and `:238` for the skip loop, genuinely require data. `ProcessFrame` derives the gravity aligned initial orientation from the first sample at `:249-250` via `FromTwoVectors(imuData->accel, Vec3::UnitZ())`, so there is no estimate to form until one exists. These must block, bounded only by a timeout so a missing publisher fails visibly.

The two preintegration sites, `:289` and `:299`, wait only to learn that the frame interval has been fully covered. They may break on an empty queue, because the block at `:307-313` already extrapolates the last available sample forward to the image stamp whenever the integration falls short. Making them non blocking removes up to one inertial period of latency from every frame, at the cost of a zero order hold whenever the queue runs dry.

A guard between the two loops is mandatory rather than defensive. `IntegratedImuMeasurement::integrate` reaches `propagateState`, which asserts `data.t_ns > curr_state.t_ns` at `thirdparty/basalt-headers/include/basalt/imu/preintegration.h:82-84`. Entering the second loop with a sample the first loop failed to advance past would integrate a stale measurement and fire that assertion.

The first blocking pop in the producer consumer lambda at `:166-174` is redundant. `ProcessFrame` acquires the first sample itself at `:224-230` under the identical guard, and `imuData` is a default constructed shared pointer and therefore null on entry in both execution models.

## `basalt_vio`, `basalt_rs_t265_vio` and `basalt_vio_sim` are already inoperative

Discovered 2026-08-23 while sweeping the blast radius of the estimator changes, and unrelated to any of them.

`VioEstimatorFactory::getVioEstimator` declares `useProducerConsumerArchitecture` with a default of false at `include/basalt/vi_estimator/vio_estimator.h:144-146`. `src/vio.cpp:311`, `src/rs_t265_vio.cpp:179` and `src/vio_sim.cpp:923` all omit that argument, so all three construct an estimator that takes the branch at `sqrt_keypoint_vio.cpp:201-205`, which registers no `proc_func` and starts no `processing_thread`. Each then wires optical flow output into `vio->vision_data_queue` and waits on `out_state_queue`, but nothing consumes the vision queue and nothing calls `ProcessFrame`, so no state is ever produced. The repair is one argument at each of the three call sites. Any reasoning about these tools' behaviour must account for the fact that they currently have none.

`src/basalt_slam.cpp` is unaffected, because it drives `Controller::TrackMonocular` directly and `Controller::initialize` passes the flag explicitly.

## `basalt_slam` feeds inertial data with an explicit lookahead

`feed_data` at `src/basalt_slam.cpp:99-150` pushes every sample up to and including the image stamp, then pushes exactly one sample beyond it, and only then calls `TrackMonocular` on the same thread. The preintegration loops therefore always terminate on their timestamp predicate rather than on an empty queue, which is why a non blocking pop is behaviour preserving for this harness.

The exception is the end of the dataset. Once `k` reaches the end of `gyro_data` the lookahead push at `:123-131` is skipped, and the second preintegration loop as currently written blocks forever. That is a latent hang in the integration test which a non blocking pop removes.

## `IngestMargData` has exactly one caller

`local_mapper.cpp:110`, inside the drain loop. Confirmed by a tree wide search. Any change to its contract is therefore a two site change and needs no compatibility shim.

## Lifecycle teardown races the sensor callbacks under the multi threaded executor

Introduced by the executor change and guarded in the same change, 2026-08-23. Recorded because the hazard is invisible under `rclcpp::spin` and will catch anyone who adds a callback group without thinking about `on_deactivate`.

`BasaltSLAMNode::on_deactivate` resets `mpFrameSubscriber`, `mpIMUSubscriber` and finally `mpSlam`. Its lifecycle callback lives in the node's default callback group, so it runs on a different executor thread from `GrabImage`. Resetting a subscription does not wait for an already dispatched callback to return, so a deactivation issued while `GrabImage` is inside `mpSlam->TrackMonocular` frees the estimator out from under it.

The guard is `std::shared_mutex mpMtxSlam` in `include/slam/basalt/node.hpp`. Both sensor callbacks take it shared and return early if `mpSlam` is null; `on_deactivate` takes it exclusively across the whole teardown. A plain mutex would be wrong, because holding it across `TrackMonocular` would serialise the two sensor callbacks again and restore the cycle the callback groups exist to break. The exclusive wait terminates because every wait inside `TrackMonocular` is now bounded.

## Remediation status, 2026-08-23

Deadlock 1 is remediated in the tree. Landed changes are the callback groups and `MultiThreadedExecutor` above, the `mpMtxSlam` guard, the split of the inertial pop into `popFromImuDataQueue` (bounded, `mpImuPopTimeoutMs` at 5000 ms, retrying every 1 ms) and `popFromImuDataQueueNonBlocking` (`try_pop`), the removal of the redundant first pop in `proc_func`, three null guards on the frame path, `try_push` on the two visualiser taps, the `break` in `SelectKeyframesToCull`, and the relocation of `IngestMargData`'s accumulator clear and `img_data` filter to its caller.

Deliberately not done, with reasons in `../plans/basalt-local-mapper-deadlock.md` section 4.3. The blocking push at `sqrt_keypoint_vio.cpp:901` keeps its backpressure, because the queue is measured never to fill and because tight VIO to mapper coupling would make a dropped packet a correctness fault rather than a quality one.

Not verified by compilation. The development container has neither `cmake` nor the `fmt` headers Sophus requires, so no build or `-fsyntax-only` pass was possible. Verification was by inspection only.

## Non blocking preintegration loses samples when the queue runs dry

The cost of the non blocking preintegration loops, recorded because it is not obvious from the code. When the second loop breaks on an empty queue, the samples arriving between the break and the image stamp are discarded by the first loop on the following frame rather than integrated, and their contribution is replaced by the zero order hold in the extrapolation block at `sqrt_keypoint_vio.cpp:307-313`. The loss is bounded by the number of samples arriving inside one image callback, and occurs only when the queue actually runs dry, which after the executor change requires the inertial publisher itself to stall.

## Deadlock 2 remains open

The shutdown sentinel described above is untouched by the 2026-08-23 work and still stands. `ext/basalt/BUG.md` remains the record.
