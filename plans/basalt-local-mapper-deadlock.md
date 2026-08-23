# Remediating the Basalt Local Mapper Deadlock under Project AirSim

## 1. Introduction

Running the `basalt` SLAM node against the Project AirSim SITL stack produces a reproducible hang. The pipeline processes frames normally for a period, and after roughly seventy five keyframes have passed through the local mapping thread the process stops making progress entirely. The node remains alive, its subscriptions remain matched, and no exception, assertion or segmentation fault is reported. Console output ceases, the published transform stops updating, and CPU occupancy falls to nil, which distinguishes the failure from an unbounded computation and identifies it as a genuine blocking deadlock rather than a livelock or a pathological slowdown.

The failure must not be confused with the shutdown deadlock recorded in `ext/basalt/BUG.md`. That document describes a hang in `Controller::Stop` during graceful teardown, caused by the local mapping thread waiting on a marginalisation sentinel that the event driven execution model never emits. The defect analysed here occurs during steady state operation, long before any shutdown is requested, and has a wholly separate mechanism. The two share only the observation that the event driven execution model inherits blocking contracts written for the producer consumer model.

This document records the architecture that produces the hang, isolates the root cause, and specifies the changes required to remove it. It also records the latent null dereferences discovered during the investigation, which are unreachable today but which the remediation would otherwise expose, and the separate finding that three offline executables in this tree are already inoperative for an unrelated reason.

## 2. System Architecture Investigated

### 2.1 Selection of the execution model

The `basalt` pipeline supports two execution models, chosen by the `useProducerConsumerArchitecture` argument to `Controller::initialize`. In the producer consumer model each stage owns a thread and communicates over bounded queues. In the event driven model no stage owns a thread, and the optical flow frontend and the visual inertial backend are invoked synchronously on whichever thread supplies the image.

`src/basalt/slam.cpp:48-55` selects the event driven model explicitly.

```cpp
mpController->initialize(0,                        // t_ns: start at origin
                         Sophus::SE3d(),           // T_w_i: identity pose
                         Eigen::Vector3d::Zero(),  // vel_w_i: zero velocity
                         bg.cast<double>(),  // gyro bias from calibration
                         ba.cast<double>(),  // accel bias from calibration
                         false,  // useProducerConsumerArchitecture = false
                         useVisualisation  // enableVisualisation
);
```

Consequently `FrameToFrameOpticalFlow` spawns no `processingLoop`, and `SqrtKeypointVioEstimator::initialize` takes the branch at `ext/basalt/src/vi_estimator/sqrt_keypoint_vio.cpp:201-205` which registers no `proc_func` and creates no `processing_thread`. `Controller::TrackMonocular` at `ext/basalt/src/controller.cpp:198-210` therefore performs the whole of optical flow and the whole of the visual inertial backend inline on its caller's thread.

### 2.2 The executor supplying that thread

`src/basalt/driver.cpp:10` drives the node with the single threaded executor.

```cpp
rclcpp::spin(node->get_node_base_interface());
```

`BasaltSLAMNode::on_activate` creates both subscriptions without specifying a callback group, so both land in the node's default mutually exclusive group. A single threaded executor dispatches one callback at a time from that group, which means `GrabImage` and `GrabIMU` are serialised onto one operating system thread. While `GrabImage` is executing, `GrabIMU` cannot run, and no amount of message arrival will change that.

The pipeline therefore contains exactly two threads of consequence. The executor thread runs image ingestion, optical flow, and the entire visual inertial backend. The mapping thread, spawned unconditionally at `ext/basalt/src/vi_estimator/local_mapper.cpp:50`, runs `LocalMapper::MapLocally`.

### 2.3 The inertial code path on the executor thread

`SqrtKeypointVioEstimator::ProcessFrame` obtains inertial samples through `popFromImuDataQueue`, defined at `ext/basalt/src/vi_estimator/sqrt_keypoint_vio.cpp:339-343`.

```cpp
SqrtKeypointVioEstimator<Scalar_>::popFromImuDataQueue() {
    ImuData<double>::Ptr data;
    this->imu_data_queue.pop(data);
```

`tbb::concurrent_bounded_queue::pop` blocks until an element is available. It offers no timeout and no cancellation. `ProcessFrame` calls it at four sites, namely `:225` for the first sample, `:238` while discarding samples that precede initialisation, and `:289` and `:299` while preintegrating across the current frame interval. The loop at `:297` is the significant one.

```cpp
while (imuData->t_ns <= curr_frame->t_ns) {
    meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
    imuData = popFromImuDataQueue();
    if (!imuData) return nullptr;
```

The predicate terminates only on a sample strictly newer than the current image timestamp, so every processed frame requires at least one inertial sample from beyond its own stamp. The queue holds three hundred samples, set at `:125`, which at the 167 Hz AP_DDS rate recorded in `ext/basalt/context/ap_dds_imu_stream.md` is approximately 1.8 seconds of buffer.

The sole producer for that queue is `Controller::GrabIMU` at `ext/basalt/src/controller.cpp:218-222`, reached from `BasaltSLAMNode::GrabIMU`, dispatched by the same executor.


### 2.4 What empties the inertial queue

A blocking pop on a queue that the blocked thread itself fills is fatal only once the queue is empty. The question that governs when the hang arrives is therefore what drains three hundred samples of buffer, and the answer lies in the middleware rather than in basalt.

`BasaltSLAMNode::on_activate` subscribes to the inertial topic with `rclcpp::SensorDataQoS()` at `src/basalt/node.cpp:127-129`, which is BEST_EFFORT, VOLATILE and KEEP_LAST with a depth of five. That depth is not a detail. While the executor thread is inside `GrabImage`, it dispatches nothing, so arriving inertial messages accumulate in the subscription's history, and a KEEP_LAST history of five retains only the five most recent and silently discards the rest. When the executor returns and drains that history, at most five samples enter `imu_data_queue` for the whole duration of the image callback.

The consumption side is fixed by the preintegration loop. `ProcessFrame` consumes every sample up to and including the current image stamp, so it removes approximately the inertial rate multiplied by the image period, which at the 166.667 Hz measured in `ext/basalt/context/ap_dds_imu_stream.md` is one sample per six milliseconds of image period.

The accounting per frame follows. Over one image period the executor is occupied inside `GrabImage` for the duration of the callback and idle for the remainder. During the idle remainder inertial messages are dispatched as they arrive and every one of them is admitted. During the callback the history caps admission at five, however many arrived. Admission per frame is therefore about five plus the idle time divided by six milliseconds, while consumption is the whole image period divided by six milliseconds. The deficit is the callback duration divided by six milliseconds, less five, which is positive for any callback exceeding thirty milliseconds and grows one sample for every additional six milliseconds of callback.

`ext/basalt/context/airsim_camera_extrinsics.md` records the measured camera rate on this stack as 7.42 Hz against a 19.6 Hz ceiling, an image period of about 135 milliseconds and therefore roughly twenty two inertial samples consumed per frame. A callback of fifty milliseconds gives a deficit of about three samples per frame, which drains three hundred samples in about a hundred frames, some thirteen seconds. That is the right order for a failure observed after tens of keyframes, and it shows how sharply the time to failure depends on callback duration, since a callback of a hundred milliseconds would drain the same buffer in about twenty five frames.

The marginalisation queue is not the trigger. `Controller::initialize` wires marginalisation output to the mapper across a ten slot queue at `ext/basalt/src/controller.cpp:172-173`.

```cpp
local_map_input_queue_.set_capacity(10);
vio_estimator_->out_marg_queue = &local_map_input_queue_;
```

`SqrtKeypointVioEstimator::marginalize` fills it with a blocking push at `ext/basalt/src/vi_estimator/sqrt_keypoint_vio.cpp:901`. Instrumentation on the running stack shows that queue reporting a size of minus one immediately before each push, which for `tbb::concurrent_bounded_queue` denotes one consumer already waiting inside `pop`, and shows the packet consumed as soon as it is deposited. The mapper keeps pace with marginalisation, the queue never approaches its capacity of ten, and the blocking push at `:901` never blocks in practice. It is a plausible culprit that the measurement rules out, and it is recorded here so that it is not proposed again.

The mapper cost defects below therefore bear on latency and on map quality rather than on the deadlock, but they remain worth correcting, because the whole starvation argument turns on how long the image callback occupies the executor and the mapper feeds back into that through `mpVioPoseUpdateCallback`.

`LocalMapper::SelectKeyframesToCull` appends a candidate inside its inner loop without terminating that loop, at `ext/basalt/src/vi_estimator/local_mapper.cpp:651-663`.

```cpp
for (size_t j = i; j < ordered.size(); ++j) {
    if (i == j) continue;
    const int64_t b = ordered[j];
    const size_t covis = ComputeCovisibility(a, b, calib.intrinsics.size());
    if (static_cast<double>(covis) / static_cast<double>(total_a) >= mpCullCovisibilityThresh)
        keyframesToCull.push_back(a);
}
```

A keyframe redundant against k others is therefore appended k times. `CullRedundantKeyframes` iterates that vector and re-executes the full rehost, `removeFrame`, factor pruning, bag of words removal and track builder cleanup sequence for keyframes it has already erased. With `mpCullCovisibilityThresh` at 0.5 and a fifty frame map, the list holds hundreds of entries where it should hold a handful, and each entry drives `FindBestRehostKf`, itself a linear scan over candidates calling `ComputeCovisibility` at every step.

`LocalMapper::IngestMargData` clears the new keyframe set on entry at `:295-296`, while the drain loop at `:108-112` invokes it once per batched packet.

```cpp
while (!vecData.empty()) {
    MargData::Ptr data = vecData.back();
    vecData.pop_back();
    IngestMargData(data);
}
```

Only the final call's keyframes survive, and because the batch is consumed from the back it is processed in reverse arrival order, so the survivors belong to the oldest packet. Every other packet's keyframes are inserted into `frame_poses` yet never tracked, matched or culled. The map grows past `mpMaxLocalMapSize`, which inflates the quadratic covisibility scan further. This is precisely the accumulation defect anticipated in `ext/basalt/local_mapper_optimisation.md` section 4.2, whose prescribed remedy was never implemented.

### 2.5 Inventory of blocking edges

| Site | Operation | Capacity | Producer or consumer thread |
|---|---|---|---|
| `sqrt_keypoint_vio.cpp:225,238,289,299` | `imu_data_queue.pop` | 300 | produced by the same executor thread that blocks here |
| `controller.cpp:220` | `imu_data_queue.push` | 300 | consumed by the same executor thread |
| `sqrt_keypoint_vio.cpp:901` | `out_marg_queue.push` | 10 | consumed by the mapping thread, measured never full |
| `sqrt_keypoint_vio.cpp:621` | `out_vis_queue.push` | 20 | consumed by the visualiser, only when enabled |
| `sqrt_keypoint_vio.cpp:594` | `out_state_queue.push` | 100 | consumed by the visualiser, only when enabled |
| `local_mapper.cpp:83` | `mpMargInputQueue.pop` | 10 | cannot observe `mpStopLocalMapping` while blocked |

The first two rows form the fatal pair. The remaining blocking pushes are latent rather than active, the third because the mapper is measured to keep pace and the fourth and fifth because the graphical interface is disabled by default.

It is worth recording that the system holds exactly one mutex, `mpPosesToUpdateMutex`. It is taken at `sqrt_keypoint_vio.cpp:369-407` inside a closed block that ends before `optimize_and_marg` runs, and at `sqrt_keypoint_vio.h:215` for a plain map copy. No lock ordering cycle exists anywhere in the pipeline, so the hang is not a mutex deadlock and no effort should be spent searching for one.

### 2.6 Stack trace at the point of failure

```
   /ap/imu/experimental/data  ──► DDS history  KEEP_LAST(5), BEST_EFFORT
                                  node.cpp:127   ONLY 5 SAMPLES SURVIVE A
                                                 BUSY EXECUTOR; THE REST ARE
                                                 SILENTLY DISCARDED
                                        │
                    ROS 2 SINGLE-THREADED EXECUTOR (the only dispatch thread)
                    rclcpp::spin  ......................  src/basalt/driver.cpp:10
                                        │
                    ┌───────────────────┴───────────────────┐
                    │  default mutually exclusive group     │
                    │  ONE callback at a time               │
                    └───────────────────┬───────────────────┘
                                        │
              ┌─────────────────────────┴─────────────────────────┐
              │                                                   │
     BasaltSLAMNode::GrabImage                          BasaltSLAMNode::GrabIMU
     node.cpp:171                                       node.cpp:197
              │                                                   │
     BasaltSLAM::TrackMonocular                         BasaltSLAM::GrabIMU
     slam.cpp:93                                        slam.cpp:106
              │                                                   │
     Controller::TrackMonocular                         Controller::GrabIMU
     controller.cpp:198                                 controller.cpp:218
              │                                                   │
     opt_flow->processFrame        (inline, no thread)   imu_data_queue.push(data)
              │                                          controller.cpp:220
     vio->ProcessFrame(res)                                       │
     controller.cpp:203                                           ▼
              │                                          ┌──────────────────┐
              ├──────────────────────────────────────►   │ imu_data_queue   │
              │                                          │ capacity 300     │
              │   popFromImuDataQueue()                   │ ~1.8 s @ 167 Hz  │
              │   sqrt_keypoint_vio.cpp:225,238,289,299   └────────┬─────────┘
              │            │                                       │
              │            └───── imu_data_queue.pop(data) ◄────────┘
              │                   sqrt_keypoint_vio.cpp:342
              │                   BLOCKING, no timeout
              │
              │   measure(curr_frame, meas)          sqrt_keypoint_vio.cpp:317
              │            │
              │   optimize_and_marg()
              │            │
              │   marginalize()
              │            │
              │   out_marg_queue->push(m)            sqrt_keypoint_vio.cpp:901
              │            │                          measured size -1 before
              ▼            ▼                          push; never actually full
       ┌──────────────────────────┐
       │ local_map_input_queue_   │   capacity 10   controller.cpp:172
       └────────────┬─────────────┘
                    │
                    ▼
     ═══════════════════════════════════════════════════════════════════
                    LOCAL MAPPING THREAD  (the only independent thread)
                    local_mapper.cpp:50
     ═══════════════════════════════════════════════════════════════════
                    │
       LocalMapper::MapLocally                        local_mapper.cpp:67
                    │
       mpMargInputQueue->pop(data)                    local_mapper.cpp:83
                    │
       IngestMargData  x N   (clears new-KF set each call — only last survives)
                    │        local_mapper.cpp:108-112, 295
       detect_keypoints / match_stereo / MatchLocal / build_tracks / setup_opt
                    │
       CullRedundantKeyframes                         local_mapper.cpp:196
                    │  └─ SelectKeyframesToCull appends duplicates  :651-663
                    │     → hundreds of redundant rehost passes
                    │     → mapper latency, fed back through the pose callback
       optimize(5) / filterOutliers / optimize(5)
                    │
       mpVioPoseUpdateCallback(frame_poses)           local_mapper.cpp:264
                    │
                    └────► back to pop()


     ─────────────────────  THE DEADLOCK SEQUENCE  ─────────────────────

     t0   Per-frame image callback cost grows past 30 ms as the map fills:
          optical flow + backend solve + marginalisation on one thread.

     t1   While the executor sits in GrabImage it dispatches nothing.
          IMU messages pile up in the DDS history, which is KEEP_LAST(5).
          Everything older than the newest five is DISCARDED by the middleware.

     t2   Per frame:  admitted ≤ 5 samples,  consumed ≈ frame_period / 6 ms.
          For any frame period above 30 ms the queue falls every frame.

     t3   The deficit accumulates. imu_data_queue drains from 300 to EMPTY.

     t4   Next image. ProcessFrame needs a sample newer than the frame stamp.
          imu_data_queue is empty  →  pop() BLOCKS.       [vio.cpp:342]

     t5   The only producer is GrabIMU, dispatched by THIS thread,
          which is the thread now blocked inside pop().

          ╔══════════════════════════════════════════════════════════╗
          ║  SELF-DEADLOCK. The thread waits on data that only it    ║
          ║  could deliver. Nothing breaks the wait. Unrecoverable.  ║
          ╚══════════════════════════════════════════════════════════╝

     The mapping thread now sits idle in pop() at local_mapper.cpp:83 with an
     empty queue. That pairing — an idle mapper alongside a blocked VIO — is
     the diagnostic signature that distinguishes this from ordinary slowness.
```

## 3. Root Cause

The root cause is that the inertial input queue is drained by the same thread that fills it. `popFromImuDataQueue` performs an unbounded blocking wait on `imu_data_queue`, and under the single threaded executor selected by `src/basalt/driver.cpp:10` the only producer for that queue is a callback which that very thread must dispatch. The wait is therefore a self deadlock by construction, and it is permanent because no timeout, cancellation or sentinel can release it.

The defect is latent rather than immediate because the three hundred sample buffer masks it, and what drains that buffer is the interaction between the subscription's history depth and the duration of the image callback. `rclcpp::SensorDataQoS()` requests KEEP_LAST with a depth of five, so a busy executor loses every inertial message beyond the newest five, while `ProcessFrame` consumes one sample per six milliseconds of image period. Whenever the image callback and the image period both exceed thirty milliseconds, admission falls below consumption, and the buffer empties at the accumulated deficit. This accounts for the observed latency to failure without recourse to any queue reaching capacity, and it is consistent with the measurement that the marginalisation queue never does.

The two mapper defects, duplicate culling candidates from `SelectKeyframesToCull` and discarded keyframe sets from the batched `IngestMargData`, are not the trigger. They lengthen the mapper cycle and degrade the map, and through `mpVioPoseUpdateCallback` they add to the work the executor thread performs per frame, so they shorten the time to failure without being its cause.

The architectural error underlying all of this is that blocking queue contracts written for the producer consumer model were inherited unchanged by the event driven model. In the producer consumer model every blocking edge has a distinct thread on each side and backpressure is the intended behaviour. In the event driven model the stages collapse onto one thread, and the same edges become cycles. `ext/basalt/BUG.md` identifies the same class of error for the shutdown sentinel, which indicates the pattern is systematic rather than incidental.

## 4. Solution

The remedy proceeds in three layers. The first removes the self deadlock by giving the inertial callback a thread of its own. The second removes the unbounded waits from the inertial path itself, so that no future scheduling change can reintroduce the cycle and so that a momentarily empty queue costs a little accuracy rather than the whole system. The third restores the mapper to its intended cost, which is a latency and map quality concern rather than a deadlock one.

### 4.1 Dedicate a callback group to the inertial subscription and adopt a multi threaded executor

This is the change that removes the deadlock. Assigning the two subscriptions to distinct mutually exclusive callback groups permits a multi threaded executor to dispatch `GrabIMU` while `GrabImage` is still executing, so the inertial queue continues to fill regardless of how long the image path runs. This also removes the starvation mechanism of section 2.4 at its source, because the subscription's five deep history is drained continuously rather than only between image callbacks. Mutually exclusive groups are chosen rather than reentrant ones because neither callback is reentrant, `GrabImage` mutating `m_cvImPtr` and `mpCurrentFrame` and `GrabIMU` mutating `mpLastIMUTimestamp`.

The callback groups must be created in the constructor rather than in `on_activate`. An executor collects a node's callback groups when the node is added, and groups created afterwards are only picked up when the executor rescans. Creating them at construction, before `add_node`, removes that ordering hazard entirely.

Add to the private section of `include/slam/basalt/node.hpp`.

```cpp
    rclcpp::CallbackGroup::SharedPtr mpImageCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr mpImuCallbackGroup;
```

Create them in the `BasaltSLAMNode` constructor in `src/basalt/node.cpp`.

```cpp
    mpImageCallbackGroup =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mpImuCallbackGroup =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

Attach them to the subscriptions in `BasaltSLAMNode::on_activate`, replacing the two `create_subscription` calls at `src/basalt/node.cpp:116-130`. The existing QoS arguments must be preserved exactly, since `context/basalt-controller-api.md` records that a bare depth requests RELIABLE and will never match the BEST_EFFORT publishers offered by the AirSim bridge and by AP_DDS.

```cpp
    rclcpp::SubscriptionOptions imageOptions;
    imageOptions.callback_group = mpImageCallbackGroup;

    mpFrameSubscriber = this->create_subscription<ImageMsg>(
        mpCameraTopicName, rclcpp::SensorDataQoS(),
        std::bind(&BasaltSLAMNode::GrabImage, this, std::placeholders::_1),
        imageOptions);

    if (mpSLAMType == eSLAMType::VISLAM) {
        rclcpp::SubscriptionOptions imuOptions;
        imuOptions.callback_group = mpImuCallbackGroup;

        mpIMUSubscriber = this->create_subscription<ImuMsg>(
            mpIMUTopicName, rclcpp::SensorDataQoS(),
            std::bind(&BasaltSLAMNode::GrabIMU, this, std::placeholders::_1),
            imuOptions);
    }
```

Replace the executor in `src/basalt/driver.cpp`. The superseded call is retained in place per section 2 of `/ws/CLAUDE.md`, since a future reader comparing against historical runs needs to see which model produced them.

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<BasaltSLAMNode> node = std::make_shared<BasaltSLAMNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

Backwards compatibility. `BasaltSLAMNode` is a lifecycle node, so the executor also dispatches its lifecycle service callbacks. Those remain in the node's default callback group and are unaffected by the two new groups. No other node in the package shares this driver, and `orbslam3` and `morbslam` have their own drivers which are untouched. The change is confined to how callbacks are scheduled, and neither callback body is modified.

One consequence must be recorded rather than left implicit. With `GrabIMU` on its own thread, `imu_data_queue.push` at `controller.cpp:220` becomes a genuine cross thread edge, and since that queue has capacity three hundred the inertial thread will block there if the backend ever falls three hundred samples behind. That is correct backpressure, because a distinct thread is on the other side and will drain it, and it is the behaviour the queue was designed for. It is nonetheless a blocking edge on a real time callback, and if inertial timestamps are later found to lag under load this is where to look.

A second consequence is a lifetime hazard that the single threaded executor had made impossible, and it is the reason this section changes more than the two `create_subscription` calls. `BasaltSLAMNode::on_deactivate` destroys the pipeline, resetting `mpFrameSubscriber`, `mpIMUSubscriber` and finally `mpSlam`. Its lifecycle callback lives in the node's default callback group, so under a multi threaded executor it runs on a different thread from `GrabImage`. Resetting a subscription does not wait for a callback already dispatched to return, so a deactivation issued while `GrabImage` is inside `mpSlam->TrackMonocular` frees the estimator out from under it. Under `rclcpp::spin` the two were serialised and the sequence could not arise.

The guard is a shared mutex rather than a plain one, because a plain mutex held across `TrackMonocular` would serialise the two sensor callbacks again and restore the very cycle section 4.1 exists to break. Both callbacks take it shared, so they remain concurrent with each other, and the teardown takes it exclusively, which waits for whichever callbacks are in flight and admits no new ones. The wait terminates because section 4.2 bounds every wait inside `TrackMonocular`.

Add to the private section of `include/slam/basalt/node.hpp`, with `#include <shared_mutex>`.

```cpp
    // Sensor callbacks hold this shared; on_deactivate takes it exclusively
    // before destroying mpSlam, so a transition cannot free the pipeline out
    // from under a callback still running on another executor thread.
    std::shared_mutex mpMtxSlam;
```

Take it at the head of both sensor callbacks in `src/basalt/node.cpp`. The null test is what makes a callback dispatched during teardown harmless rather than fatal.

```cpp
void BasaltSLAMNode::GrabImage(const ImageMsg::SharedPtr msg) {
    std::shared_lock<std::shared_mutex> slamLock(mpMtxSlam);
    if (!mpSlam) return;
```

```cpp
void BasaltSLAMNode::GrabIMU(const ImuMsg::SharedPtr msg) {
    std::shared_lock<std::shared_mutex> slamLock(mpMtxSlam);
    if (!mpSlam) return;
```

Take it exclusively across the whole of the teardown in `BasaltSLAMNode::on_deactivate`, after the base class transition has run.

```cpp
    CallbackReturn result = SlamNode::on_deactivate(previous_state);

    // Exclusive for the whole teardown: under the multi-threaded executor a
    // sensor callback may still be inside mpSlam on another thread, and
    // resetting a subscription does not wait for it to return.
    std::unique_lock<std::shared_mutex> slamLock(mpMtxSlam);

    mpSlam->StopSlamVisualiser();
```

### 4.2 Restructure the inertial pop into a blocking and a non blocking form

Section 4.1 removes the deadlock for the current node. This section removes it from the estimator itself, so that no caller can reconstruct it, and in doing so removes a latency the current design imposes on every frame.

The design rests on a distinction the present code does not draw. Two of the four pop sites genuinely require data and cannot proceed without it, while the other two are waiting only to learn that the interval has been fully covered, and can equally well conclude that it has been covered as far as the data extends.

Initialisation is the case that must wait. `ProcessFrame` derives the gravity aligned initial orientation from the first inertial sample at `sqrt_keypoint_vio.cpp:249-250`, through `Eigen::Quaternion<Scalar>::FromTwoVectors(imuData->accel, Vec3::UnitZ())`, and there is no meaningful estimate to form until a sample exists. The pop at `:225` that acquires the first sample, and the skip loop at `:238` that advances to the first sample at or beyond the first image, must therefore remain blocking. They are bounded by a timeout only so that a stack brought up without an inertial publisher fails visibly rather than hanging.

Preintegration is the case that need not wait. The two loops at `:288-305` each terminate on a timestamp condition that only the arrival of a future sample can satisfy.

```cpp
        while (imuData->t_ns <= this->prev_frame->t_ns) { ... }

        while (imuData->t_ns <= curr_frame->t_ns) {
            meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
            ...
        }
```

The second loop exits only on a sample strictly newer than the current image stamp, so every frame is held until inertial data from beyond its own timestamp has arrived. Under a 167 Hz stream that is a wait of up to six milliseconds imposed on the image path for no purpose other than closing the integration interval exactly, and the code that follows already handles an interval closed inexactly. The block at `:307-313` extrapolates the last available sample forward to the image stamp precisely when the integration falls short.

```cpp
        if (meas->get_start_t_ns() + meas->get_dt_ns() < curr_frame->t_ns) {
            if (!imuData.get()) return nullptr;
            int64_t tmp = imuData->t_ns;
            imuData->t_ns = curr_frame->t_ns;
            meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
            imuData->t_ns = tmp;
        }
```

Breaking both loops when the queue is empty therefore yields a preintegrated measurement that still spans the full frame interval, closed by a zero order hold on the last sample rather than by the true next sample. On the following frame the first loop advances past the samples that arrived in the interim and the second resumes integrating, so the scheme is self correcting.

Two implementation methods replace the single one. Declare them in `ext/basalt/include/basalt/vi_estimator/sqrt_keypoint_vio.h`, alongside the timeout that bounds the blocking form.

```cpp
    typename ImuData<Scalar>::Ptr popFromImuDataQueue();
    bool popFromImuDataQueueNonBlocking(typename ImuData<Scalar>::Ptr& data);

    int64_t mpImuPopTimeoutMs = 5000;
    static constexpr int64_t mpImuPopRetryMs = 1;
```

Replace the body of `popFromImuDataQueue` at `ext/basalt/src/vi_estimator/sqrt_keypoint_vio.cpp:337-354`. The blocking form is expressed in terms of the non blocking one, retrying every millisecond until the deadline.

```cpp
template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::popFromImuDataQueueNonBlocking(
    typename ImuData<Scalar>::Ptr& data) {
    ImuData<double>::Ptr raw;
    if (!this->imu_data_queue.try_pop(raw)) return false;

    if constexpr (std::is_same_v<Scalar, double>) {
        data = raw;
    } else {
        typename ImuData<Scalar>::Ptr converted;
        if (raw) {
            converted.reset(new ImuData<Scalar>);
            *converted = raw->cast<Scalar>();
        }
        data = converted;
    }
    return true;
}

template <class Scalar_>
typename ImuData<Scalar_>::Ptr
SqrtKeypointVioEstimator<Scalar_>::popFromImuDataQueue() {
    typename ImuData<Scalar>::Ptr data;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(mpImuPopTimeoutMs);

    while (!popFromImuDataQueueNonBlocking(data)) {
        if (std::chrono::steady_clock::now() >= deadline) return nullptr;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(mpImuPopRetryMs));
    }
    return data;
}
```

Note that `popFromImuDataQueueNonBlocking` returns true and assigns a null pointer when it pops the shutdown sentinel, which is the distinction the return value exists to draw. A false return means the queue was empty and `data` is untouched, so a caller may safely keep the sample it already holds.

Remove the first blocking pop in the producer consumer lambda at `ext/basalt/src/vi_estimator/sqrt_keypoint_vio.cpp:166-174` entirely. It is redundant, because `ProcessFrame` acquires the first sample itself at `:224-230` under the identical guard, and `imuData` is a default constructed shared pointer and therefore null on entry in both execution models.

```cpp
        auto proc_func = [&] {
            OpticalFlowResult::Ptr curr_frame;

            // Superseded 2026-08-23: the first blocking pop and its assertion
            // duplicated ProcessFrame's own acquisition at :224-230.
            // this->imuData = popFromImuDataQueue();
            // BASALT_ASSERT_MSG(imuData, "first IMU measurment is nullptr");
            // ... bias calibration of that first sample ...

            while (true) {
```

Guard the first acquisition in `ProcessFrame` at `:224-230`, which currently dereferences the result unconditionally and which the timeout makes reachable.

```cpp
    if (this->imuData == nullptr) {
        this->imuData = popFromImuDataQueue();
        if (!this->imuData) return nullptr;
        this->imuData->accel =
            this->calib.calib_accel_bias.getCalibrated(imuData->accel);
        this->imuData->gyro =
            this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
    }
```

Guard the initialisation branch at `:236-251`, which breaks out of its loop on a null sample and then dereferences that same pointer to derive the gravity aligned orientation.

```cpp
    if (!initialized) {
        while (imuData->t_ns < curr_frame->t_ns) {
            imuData = popFromImuDataQueue();
            if (!imuData) break;
            imuData->accel =
                this->calib.calib_accel_bias.getCalibrated(imuData->accel);
            imuData->gyro =
                this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
        }

        if (!imuData) return nullptr;

        Vec3 vel_w_i_init;
```

Replace the two preintegration loops at `:288-305` with the non blocking form. The guard on the second loop is essential rather than defensive. `IntegratedImuMeasurement::integrate` reaches `propagateState`, which asserts `data.t_ns > curr_state.t_ns` at `thirdparty/basalt-headers/include/basalt/imu/preintegration.h:82-84`, so entering the second loop with a sample the first loop failed to advance past would integrate a stale measurement and fire that assertion.

```cpp
        bool imuAhead = imuData->t_ns > this->prev_frame->t_ns;
        while (!imuAhead) {
            if (!popFromImuDataQueueNonBlocking(imuData)) break;
            if (!imuData) return nullptr;
            imuData->accel =
                this->calib.calib_accel_bias.getCalibrated(imuData->accel);
            imuData->gyro =
                this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
            imuAhead = imuData->t_ns > this->prev_frame->t_ns;
        }

        if (imuAhead) {
            while (imuData->t_ns <= curr_frame->t_ns) {
                meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
                if (!popFromImuDataQueueNonBlocking(imuData)) break;
                if (!imuData) return nullptr;
                imuData->accel =
                    this->calib.calib_accel_bias.getCalibrated(imuData->accel);
                imuData->gyro =
                    this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
            }
        }
```

`Controller::TrackMonocular` dereferences the returned state without a check at `ext/basalt/src/controller.cpp:203-204`, and `ProcessFrame` can already return null today at lines 290, 300 and 308. This is an existing latent segmentation fault which the timeout and the null propagation above make materially more likely to be reached.

```cpp
    current_latest_pose_ = vio_estimator_->ProcessFrame(res);
    if (current_latest_pose_) {
        tcw = current_latest_pose_->T_w_i.cast<float>();
    }
```

The accuracy cost must be stated plainly. When the second loop breaks on an empty queue, the samples that arrive between the break and the image stamp are discarded by the first loop on the following frame rather than integrated, and their contribution is replaced by the zero order hold in the extrapolation block. The loss is bounded by the number of samples arriving inside one image callback and occurs only when the queue actually runs dry, which after section 4.1 requires the inertial publisher itself to stall. Against that, every frame is relieved of a wait of up to one inertial period, and the frame path is no longer coupled to inertial arrival at all.

Backwards compatibility. This section deliberately does not preserve the previous behaviour behind a default, because the previous behaviour is the defect and, as section 4.7 establishes, no working caller depends on it. `mpImuPopTimeoutMs` is a member with a five second default rather than a compile time constant so a caller may lengthen it, and the retry interval is fixed at one millisecond as a matter of policy. The three null guards alter behaviour only along paths that are undefined today, replacing a dereference of a null pointer with an early return, so no defined behaviour changes. `TrackMonocular` leaves `tcw` untouched on a dropped frame, which its callers already tolerate because the pose is only read after the call returns.

### 4.3 Retain blocking backpressure on the marginalisation push

Dropping marginalisation packets on a full queue is the obvious companion to the other changes in this section, on the reasoning that the mapper is advisory and a lost packet is cheaper than a stalled pipeline. It is deliberately not done, and the reasons are recorded here so it is not proposed again.

The measurement rules out the premise. Instrumentation on the running stack reports `local_map_input_queue_` at a size of minus one immediately before each push, which for `tbb::concurrent_bounded_queue` denotes a consumer already parked inside `pop`, and the packet is taken as soon as it is deposited. The queue never approaches its capacity of ten, so the blocking push at `sqrt_keypoint_vio.cpp:901` never blocks, and a mechanism that never engages cannot be made safer by weakening it.

The intended direction of the design rules it out independently. The mapper's role today is advisory, its corrections reaching only the already marginalised `frame_poses` as established in `ext/basalt/context/vio_localmapper_correction_loop.md`, but the plan of record is to couple the mapper tightly to the backend. Under tight coupling a marginalisation packet is not advisory at all, and silently discarding one would corrupt the very state the coupling exists to share. Building a drop path now would install a defect that the next architectural step would have to remove.

The push therefore remains as it is. Should the queue ever be observed at capacity, the correct response is to reduce mapper cycle time by way of sections 4.4 and 4.5, not to discard the mapper's input.

### 4.4 Emit each culling candidate once

`SelectKeyframesToCull` appends a keyframe once for every other keyframe it is redundant against, and `CullRedundantKeyframes` then repeats the entire removal sequence for a frame it has already erased. Terminating the inner loop on the first qualifying partner reduces the candidate list to the set it was always intended to hold, and eliminates the repeated rehosting that dominates mapper cycle time on a saturated map.

Amend the inner loop at `ext/basalt/src/vi_estimator/local_mapper.cpp:651-663`.

```cpp
        for (size_t j = i; j < ordered.size(); ++j) {
            if (i == j) continue;
            const int64_t b = ordered[j];
            const size_t covis =
                ComputeCovisibility(a, b, calib.intrinsics.size());
            if (static_cast<double>(covis) / static_cast<double>(total_a) >=
                mpCullCovisibilityThresh) {
                keyframesToCull.push_back(a);
                break;
            }
        }
```

Backwards compatibility. `SelectKeyframesToCull` is private to `LocalMapper` and is called from exactly one site, `CullRedundantKeyframes` at `local_mapper.cpp:821`. The set of distinct keyframes selected for culling is unchanged, since a keyframe qualified on its first redundant partner and every subsequent append named the same frame. Only the multiplicity changes, so this removes repeated work without altering which frames survive.

### 4.5 Accumulate new keyframes across a drained batch

`IngestMargData` clears the tracking set on entry, so a drained batch retains only the last packet's keyframes, and reverse consumption makes those the oldest. The remedy prescribed in `ext/basalt/local_mapper_optimisation.md` section 4.2 is external accumulation, which is adopted here.

`IngestMargData` has exactly one caller, the drain loop at `local_mapper.cpp:108-112`, confirmed by a tree wide search for the symbol. Rather than carry the new behaviour on an optional argument whose default reproduces the old one, the method is changed to accumulate unconditionally and its single caller is updated to match. This leaves one behaviour in the tree rather than two, which is the right trade when the set of callers is one and the old behaviour is a defect.

Amend the signature in `ext/basalt/include/basalt/vi_estimator/local_mapper.h`. It is unchanged in shape, and only its contract moves, from resetting the accumulators to appending to them.

```cpp
    // Ingests one marginalisation packet, appending its new keyframes to
    // mpNewKeyframesForTracking. The caller clears the accumulators before a
    // batch and filters img_data after it.
    void IngestMargData(MargData::Ptr& data);
```

Remove the entry clear at `ext/basalt/src/vi_estimator/local_mapper.cpp:295-296` and the image filter at `:343-349`, both of which move to the caller.

```cpp
void LocalMapper::IngestMargData(MargData::Ptr& data) {
    // Superseded 2026-08-23: clearing here discarded every packet's keyframes
    // but the last when the drain loop batched more than one.
    // mpNewKeyframesForTracking.clear();
    // mpLatestKeyframesMatches.clear();

    // Step 1 — detect new KFs (those in kfs_all but not yet in frame_poses).
```

```cpp
    // Step 4 — the img_data filter moved to the caller, so that images
    // belonging to earlier packets in a batch survive until the batch ends.
    // for (auto it = img_data.begin(); it != img_data.end();) {
    //     if (mpNewKeyframesForTracking.count(it->first) == 0) {
    //         it = img_data.erase(it);
    //     } else
    //         ++it;
    // }
}
```

Replace the drain loop at `ext/basalt/src/vi_estimator/local_mapper.cpp:108-112` so the batch is consumed in arrival order, the accumulators are cleared once per batch, and the image filter runs once at the end.

```cpp
        // Superseded 2026-08-23: consumed the batch from the back, so packets
        // were ingested newest-first and IngestMargData's entry-clear left only
        // the OLDEST packet's keyframes in mpNewKeyframesForTracking.
        // while (!vecData.empty()) {
        //     MargData::Ptr data = vecData.back();
        //     vecData.pop_back();
        //     IngestMargData(data);
        // }

        mpNewKeyframesForTracking.clear();
        mpLatestKeyframesMatches.clear();
        for (MargData::Ptr& packet : vecData) {
            IngestMargData(packet);
        }
        vecData.clear();

        for (auto it = img_data.begin(); it != img_data.end();) {
            if (mpNewKeyframesForTracking.count(it->first) == 0) {
                it = img_data.erase(it);
            } else
                ++it;
        }
```

Note that `IngestMargData` populates `mpNewKeyframesForTracking` by testing membership in `frame_poses` and adds those frames to `frame_poses` later in the same call, so a keyframe appearing in two packets of one batch is reported new by the first call only. Accumulation across a batch is therefore free of duplicates by construction, and `mpNewKeyframesForTracking` being a `std::set` makes it so regardless.

Backwards compatibility. The single production caller is amended in the same change, so no caller is left on the old contract. `mpLatestKeyframesMatches` is cleared alongside the keyframe set as before, and `CollectNewKeyframesAfterMatching` repopulates it from `feature_matches` after `IngestMargData` has run, so its lifetime is unaffected. Consuming the batch forwards rather than backwards restores the arrival ordering that `extractNonlinearFactors` assumes, since a later packet's poses should overwrite an earlier packet's rather than the reverse. The `vecData.clear()` is retained only so the batch vector's lifetime reads the same as before, since the range loop no longer empties it as the old `pop_back` loop did.

### 4.6 Honour the non-blocking contract on the visualiser taps

This change is not required to fix the reported hang, because `use_visualisation` defaults to false at `src/basalt/node.cpp:43` and the two queues are left unwired in that configuration. It is made because it is the same defect in a third location and produces the same symptom whenever the graphical interface is enabled.

`src/visualisation/visualiser.cpp:36-37` states the contract explicitly, that producers use `try_push` so a full queue drops the newest frame instead of blocking the estimator threads. `SqrtKeypointVioEstimator` violates it at `sqrt_keypoint_vio.cpp:594` and `:621`, where blocking pushes target queues of capacity 100 and 20 respectively. `LocalMapper` honours it correctly at `local_mapper.cpp:252`.

```cpp
    // sqrt_keypoint_vio.cpp:594
    // Superseded 2026-08-23: blocking push contradicted the contract stated at
    // visualisation/visualiser.cpp:36-37 and stalled VIO on a slow GUI thread.
    // this->out_state_queue->push(data);
    this->out_state_queue->try_push(data);
```

```cpp
    // sqrt_keypoint_vio.cpp:621
    // this->out_vis_queue->push(data);
    this->out_vis_queue->try_push(data);
```

The impact of dropping on each queue was investigated separately, because the two carry different payloads and their consumers use them differently.

`mvpVioVisQueue`, of capacity twenty, carries `VioVisualizationData`, a full point cloud with projections. `SlamVisualiser::ConsumeVioVisQueue` stores each item into `mpLatestVio` under `mpMtxVioVis`, and `Run` reads only that single latest pointer at `visualiser.cpp:196-200`. The queue is therefore a latest wins channel already, and a dropped item is invisible in the rendered scene beyond a frame of staleness. This is the payload for which dropping is unambiguously correct, and it is also the expensive one, since twenty point clouds is where the memory sits.

`mvpVioStateQueue`, of capacity one hundred, carries `PoseVelBiasState` and is accumulated rather than replaced. Its consumer appends to the drawn trajectory and to the thirteen column plotter log whose layout is recorded at `visualiser.cpp:410-411`. A dropped state is therefore a permanent gap, a missing vertex in the trajectory polyline and a missing row in the plots. The cost is cosmetic, since neither structure feeds the estimate, and one hundred states is several seconds of buffer that only a stalled Pangolin thread can exhaust. One item does carry more weight than the rest, namely the first, from which `ConsumeVioStateQueue` captures `mpSlamFirstPose` for ground truth alignment as recorded in `context/gt_slam_alignment.md`. That item cannot be dropped, because the queue is necessarily empty when it is pushed.

Against those costs stands the failure the blocking form permits. With the visualiser enabled, a Pangolin thread that stutters for longer than the queue holds will block the pushing thread inside `push`, and in the event driven model that thread is the executor thread inside `GrabImage`. That is the identical starvation described in section 2.4, arriving by a different route. The graphical interface must not be able to deadlock the estimator, and a gap in a plot is the correct price for that guarantee.

The shutdown sentinel pushes at `:192-194` and `:217-219` must remain blocking. A dropped sentinel would leave a consumer waiting forever, which is the failure already documented in `ext/basalt/BUG.md`.

### 4.7 Blast radius across the other executables

The estimator changed in section 4.2 is shared, so every executable that instantiates it was examined. The finding is that only two consumers are live, and that the offline tools are already inoperative for an unrelated reason.

`VioEstimatorFactory::getVioEstimator` declares `useProducerConsumerArchitecture` with a default of false at `ext/basalt/include/basalt/vi_estimator/vio_estimator.h:144-146`. `src/vio.cpp:311`, `src/rs_t265_vio.cpp:179` and `src/vio_sim.cpp:923` all call it without supplying that argument, so all three construct an estimator that takes the branch at `sqrt_keypoint_vio.cpp:201-205`, which registers no `proc_func` and starts no `processing_thread`. Each of those tools then wires the optical flow output to `vio->vision_data_queue` and waits for states to appear on `out_state_queue`, but nothing consumes the vision queue and nothing calls `ProcessFrame`, so no state is ever produced. `basalt_vio`, `basalt_rs_t265_vio` and `basalt_vio_sim` are therefore already broken in the current tree, by the refactor that introduced the flag rather than by anything done here. They are recorded as a separate defect and are deliberately not repaired here, since repairing them is a change to tools this work does not touch. The one line repair, passing true at each of those three call sites, is noted so a future session does not have to re-derive it.

The consequence for section 4.2 is that the natural worry, that converting the inertial pop to a non blocking form might change semantics those offline tools rely on, does not arise. They exercise none of the modified code. Once they are repaired they will exercise it as genuine producer consumer callers, with a dedicated inertial feeding thread, under which the queue runs dry only if that thread falls behind and the break on empty behaves exactly as it does for the node.

`src/basalt_slam.cpp` is the live integration test and it is unaffected, which the feeding pattern makes structural rather than incidental. `feed_data` at `:99-150` pushes every inertial sample up to and including the image stamp, then pushes exactly one sample beyond it as an explicit lookahead at `:123-131`, and only then calls `TrackMonocular` on the same thread. The preintegration loops consequently always terminate on their timestamp predicate with the lookahead sample in hand, never on an empty queue, so the non blocking form takes precisely the path the blocking form took. The initialisation pops are equally safe, since `GrabIMU` has already run before the first `TrackMonocular`.

There is one place where the integration test changes behaviour, and it changes for the better. At the end of the dataset, when `k` has reached the end of `gyro_data`, the lookahead push at `:123-131` is skipped, and the blocking form of the second preintegration loop waited forever on a queue that would never receive another sample. The non blocking form breaks out, extrapolates through the block at `:307-313`, and the run terminates normally. That is a latent hang in the integration test which this work incidentally removes.

The remaining executables need no consideration. `basalt_mapper`, `basalt_mapper_sim` and `basalt_mapper_sim_naive` drive `NfrMapper` directly and never construct `SqrtKeypointVioEstimator`. `basalt_calibrate`, `basalt_calibrate_imu`, `basalt_opt_flow`, `basalt_time_alignment`, `basalt_kitti_eval` and `basalt_rs_t265_record` touch neither the estimator nor the mapper. Section 4.5 is confined to `LocalMapper`, which only `Controller` instantiates, so it reaches `basalt_slam` and the ROS node and nothing else.

## 5. Validation

The hypothesis is confirmed by attaching to the hung process and inspecting two frames.

```
gdb -p $(pgrep -f basalt) -batch -ex "thread apply all bt"
```

The expected signature is the executor thread inside `tbb::concurrent_bounded_queue::pop` beneath `popFromImuDataQueue` and `ProcessFrame`, together with the mapping thread idle in `pop` at `local_mapper.cpp:83`. An idle mapper alongside a blocked backend proves the queue is empty and therefore that the stall is the inertial wait rather than mapper latency.

One measurement taken before the change would settle the starvation argument of section 2.4 and is worth capturing if the failure can be reproduced on the pre-change binary. It is the `[VIO] IMU Data Queue Size` trace already present at `sqrt_keypoint_vio.cpp:326`, whose monotonic decline from three hundred, at a rate matching the per frame deficit predicted from the measured image period, is the direct evidence for the mechanism. The image period itself is already measured at 7.42 Hz in `ext/basalt/context/airsim_camera_extrinsics.md` and needs no fresh capture.

After the changes, the checks are as follows. The node must run past the point of previous failure with `use_visualisation:=false`, which exercises sections 4.1, 4.2, 4.4 and 4.5. The same queue size trace should show the queue oscillating around a small occupancy rather than falling monotonically, since a dedicated inertial thread both fills it continuously and is drained continuously. Mapper cycle time is already instrumented behind `mpVioDebugMode` throughout `MapLocally`, and the `CullRedundantKeyframes` timer is the one to watch, since it carries the cost that sections 4.4 and 4.5 target. The count of keyframes reported by the mapper per batch should rise from one packet's worth to the batch's worth, which is the direct observable for section 4.5.

Section 4.6 is exercised only with `use_visualisation:=true`, where the check is that the trajectory and plots remain continuous under normal load, confirming the queues are not being driven to capacity in ordinary operation.

The integration test `basalt_slam` should be run against a EuRoC sequence before and after, and the trajectories compared. Section 4.7 argues the code path is identical for that harness, so any divergence beyond floating point noise would falsify that argument and must be investigated rather than accepted.

These checks require a running SITL stack or a dataset and cannot be executed in the development container, so they are recorded here for execution against the live system.

What was actually run at implementation time is limited, and the limitation is a property of the container rather than of the changes. The development container has neither `cmake` nor the `fmt` headers that Sophus requires, so neither a package build nor a `-fsyntax-only` pass over the modified translation units is possible in it. Verification was therefore by inspection against the following specific hazards, each of which was checked and found sound.

The two sensor callbacks were read in full and share no mutable node state, `GrabImage` touching only `m_cvImPtr`, `mpCurrentFrame` and `mpTwc` and `GrabIMU` touching only `mpLastIMUTimestamp`, so mutually exclusive groups suffice and no further guard is needed between them. `Controller::GetLatestPose` and `Controller::TryPopPose`, which take `pose_mutex_`, have no callers anywhere outside `Controller` itself, so the unguarded write to `current_latest_pose_` in `TrackMonocular` is not made racy by the new threading. `rclcpp::executors::MultiThreadedExecutor` is reachable through `rclcpp/rclcpp.hpp`, which `driver.cpp` already includes. `std::this_thread::sleep_for` is reachable through the `<thread>` include already present in `sqrt_keypoint_vio.h`, and `<chrono>` is already included in the corresponding source. `mpImuPopRetryMs` is odr-used by reference through the `std::chrono::milliseconds` constructor, which is well formed because `CMakeLists.txt:96` sets C++17 and static constexpr data members are implicitly inline from that standard onward. The relocated clear of `mpLatestKeyframesMatches` was checked against the order of `MapLocally`, where the caller now clears at `:117`, `CollectNewKeyframesAfterMatching` repopulates at `:177` and the consumers at `:493` and `:518` follow, so the container is never read between the clear and its refill.

A compile and a run against a dataset remain outstanding and are the first thing to do on a machine that has the toolchain.

## 6. Conclusion

The hang is a self deadlock rather than a resource contention problem. `popFromImuDataQueue` waits without bound on a queue whose only producer is a callback that the waiting thread must itself dispatch, and the single threaded executor in `src/basalt/driver.cpp` is what places both on one thread. The three hundred sample buffer conceals the defect until the buffer empties, and what empties it is the five deep KEEP_LAST history on the inertial subscription, which discards every message beyond the newest five while the executor is occupied inside the image callback. Once the image callback exceeds thirty milliseconds each frame admits fewer samples than it consumes, and the deficit accumulates until the queue is empty and the next pop never returns. At the measured 7.42 Hz image rate a fifty millisecond callback drains the buffer in about a hundred frames.

The remedy separates the two callbacks onto distinct threads, which both removes the cycle and removes the starvation that drives the system into it, and then removes the unbounded waits from the estimator so that no future scheduling change can reconstruct the failure. The preintegration loops are additionally relieved of their dependence on a future inertial sample, which removes up to one inertial period of latency from every frame at the cost of a zero order hold when the queue runs dry. The mapper corrections address latency and map quality rather than the deadlock, which the marginalisation queue measurement establishes was never the trigger.

The wider lesson is that the event driven execution model inherited blocking queue contracts written for the producer consumer model. Under the producer consumer model every blocking edge separates two threads and backpressure is correct. Under the event driven model the stages collapse onto one thread and those same edges close into cycles. `ext/basalt/BUG.md` records the identical class of error for the shutdown sentinel, so any future work that introduces a queue into this pipeline should establish which thread sits on each side of it in both execution models before choosing between `push` and `try_push`.

A second lesson concerns the middleware rather than basalt. A history depth chosen for a sensor stream that is consumed promptly becomes a silent data loss mechanism the moment the consumer is occupied elsewhere, and BEST_EFFORT means that loss is neither reported nor recoverable. The correct fix is a dedicated thread for the callback, as in section 4.1, rather than a deeper history, which would only lengthen the interval before the same failure.
