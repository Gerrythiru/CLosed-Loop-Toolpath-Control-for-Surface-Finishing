# Proposal A — Detailed Technical Plan
## Closed-Loop Geometry-Aware Toolpath Replanning for Fixtureless Robotic Sanding under Continuous Workpiece Drift
### Validated in Isaac Sim (Simulation-Only Master's Thesis)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Manufacturing Task Definition](#2-manufacturing-task-definition)
   - 2.1 Workpiece and Process Specification
   - 2.2 Operating Envelope and Disturbance Model
   - 2.3 Failure Mode Taxonomy
3. [Process Flow](#3-process-flow)
4. [Quantitative Acceptance Criteria](#4-quantitative-acceptance-criteria)
5. [System Architecture](#5-system-architecture)
   - 5.1 Architectural Overview
   - 5.2 ROS 2 Node Graph
   - 5.3 Topics
   - 5.4 Services
   - 5.5 Actions
   - 5.6 TF Tree
   - 5.7 Node-by-Node Descriptions
   - 5.8 Interface Contracts Between Subsystems
6. [Algorithm Specifications](#6-algorithm-specifications)
   - 6.1 TSDF Volume Management (Perception Layer)
   - 6.2 Dual-Space Error-State Kalman Filter (DS-ESKF)
   - 6.3 NURBS Toolpath Planner (Planning Layer)
   - 6.4 Admittance Controller (Motion Layer)
   - 6.5 Preston-Law Material Removal Proxy
7. [Implementation Milestones](#7-implementation-milestones)
8. [Risk Register and Mitigation](#8-risk-register-and-mitigation)
9. [Thesis Contribution Statement](#9-thesis-contribution-statement)

---

## 1. Executive Summary

This plan specifies a simulation-only Master's thesis that closes the most significant remaining gap
in the robotic finishing literature: **in-pass, geometry-aware toolpath replanning on a free-floating
workpiece under continuous abrasive contact**, with no pause or reset when the workpiece drifts.

The system integrates four components that each have published precedents but have never been
combined in the manufacturing context:

- A live **TSDF surface map** of the workpiece maintained at 30 Hz from an external simulated
  RGB-D camera (Open3D KinectFusion-class pipeline).
- A **dual-NURBS toolpath planner** (Wang et al., 2022) that re-solves the path on each
  updated mesh, warm-started from the previous solution so replanning takes under 100 ms.
- A **multi-rate DS-ESKF** (Wen & Pagilla, 2023) that bridges 30 Hz camera observations with
  a 1 kHz robot-control loop via encoder-aided propagation.
- An **admittance controller** that tracks the replanned path in the live workpiece frame while
  maintaining constant normal force — using a Preston-law proxy to accumulate the material
  removal estimate that is fed back into the TSDF.

The simulator is NVIDIA Isaac Sim (PhysX 5 contact dynamics, native RGB-D rendering, ROS 2 bridge).

---

## 2. Manufacturing Task Definition

### 2.1 Workpiece and Process Specification

| Parameter | Value |
|---|---|
| **Workpiece** | 300 mm × 200 mm × 10 mm aluminum alloy 6061-T6 panel |
| **Target surface finish** | Ra ≤ 1.6 µm (equivalent to P120 grit sanding) |
| **Resting surface** | PTFE-coated table (simulated µ_static ≈ 0.04, µ_kinetic ≈ 0.02) |
| **Robot** | Universal Robots UR10e, 6-DOF, 10 kg payload |
| **End-effector** | Simulated orbital disc sander, 80 mm contact diameter |
| **Normal force setpoint** | F_n = 8 N ± 1 N |
| **TCP sanding speed** | v_contact = 50 mm/s along toolpath |
| **Abrasive grit** | P120 equivalent (Preston constant k_p = 2.5 × 10⁻¹² Pa⁻¹) |
| **External camera** | Simulated Intel RealSense D435i (640×480, 30 Hz, depth range 0.3–3 m) |
| **Camera mount** | Stationary, rigidly attached to world frame, angled 30° above horizontal |

**Why this workpiece and robot:** The UR10e is fully supported by the `ur_robot_driver` ROS 2
package including MoveIt 2 integration. The aluminum panel is large enough that toolpath
replanning involves a non-trivial path graph (≥ 200 waypoints per pass) while being small
enough that TSDF voxel memory is bounded well under 2 GB.

### 2.2 Operating Envelope and Disturbance Model

**Workpiece drift sources in the real world** (modeled explicitly in simulation):

| Source | Max displacement | Direction | Frequency content |
|---|---|---|---|
| Sanding reaction force | 5–15 mm | Lateral (XY) | Quasi-static (< 1 Hz) |
| External bump (accidental) | Up to 20 mm | XY, any direction | Impulse |
| Rocking on table | ± 3° | About Z-axis | Quasi-static |
| Out-of-plane lift | Not expected | — | — |

**Simulation disturbance injection:**
The Isaac Sim workpiece body is given a scripted external force/torque disturbance signal that
is the sum of:
1. A quasi-static drift at 0.5–2 mm/s in a random XY direction (models sanding reaction).
2. An impulse of magnitude 0.5–2 N lasting 50 ms every 15–60 s (models accidental bumps).
3. A slow Z-rotation of ±3° over 30 s (models rotational drift).

Disturbance magnitudes are drawn uniformly from these ranges at the start of each simulated trial.

### 2.3 Failure Mode Taxonomy

The following conditions constitute a trial failure (intervention required):

| Code | Condition | Threshold |
|---|---|---|
| F1 | Lost contact — TCP lifts off surface | F_n < 1 N for > 500 ms |
| F2 | Excessive contact force — risk of part damage | F_n > 25 N |
| F3 | Toolpath divergence — TCP strays from surface | Cartesian error > 10 mm |
| F4 | Pose estimation divergence — ESKF covariance explodes | trace(P) > threshold |
| F5 | TSDF registration failure — ICP fails to converge | ICP fitness < 0.6 |
| F6 | Replanning timeout | No updated waypoints within 200 ms of trigger |

---

## 3. Process Flow

The full sanding cycle is orchestrated by a hierarchical state machine with six states.
State transitions are shown below. Each state's entry conditions, actions, and exit conditions
are specified.

```
┌─────────────┐
│   IDLE      │  Waiting for operator START command
└──────┬──────┘
       │ /task/start service call
       ▼
┌─────────────┐
│  INIT_SCAN  │  External camera acquires 5 frames → TSDF fused →
│             │  ICP registration → workpiece_nominal_frame established →
│             │  Initial toolpath generated
└──────┬──────┘
       │ InitialScan action succeeds + initial toolpath ready
       ▼
┌─────────────┐
│  APPROACH   │  Robot moves from home to pre-contact pose (50 mm above
│             │  first waypoint) in joint space via MoveIt 2 plan
└──────┬──────┘
       │ TCP within 5 mm of first approach pose
       ▼
┌──────────────────────────────────────────────────────────────┐
│  CONTACT_ESTABLISH                                           │
│  Admittance controller lowers TCP at 5 mm/s in -Z           │
│  until F_n ≥ 7 N                                            │
└──────┬───────────────────────────────────────────────────────┘
       │ F_n in [7, 9] N for 200 ms
       ▼
┌──────────────────────────────────────────────────────────────┐
│  SANDING  (primary loop — runs until coverage ≥ 95%)        │
│                                                              │
│  250 Hz: Admittance controller tracks current waypoint       │
│          in workpiece_live_frame; issues joint velocity cmds │
│                                                              │
│  30 Hz:  TSDF integrates new depth frame                     │
│          ICP re-estimates workpiece_live_frame pose          │
│          DS-ESKF updates with ICP measurement                │
│          If pose error > 2 mm → trigger REPLAN               │
│          Material removal proxy updates roughness map        │
│                                                              │
│  1 kHz:  DS-ESKF propagates workpiece pose via encoder data  │
│                                                              │
│  On REPLAN trigger:                                          │
│    1. Extract current mesh from TSDF                         │
│    2. Re-solve NURBS path warm-started from previous control │
│       points                                                 │
│    3. Stream updated waypoints to motion executor            │
│    4. Motion executor resumes from nearest waypoint on       │
│       new path (no pause, no restart)                        │
└──────┬───────────────────────────────────────────────────────┘
       │ coverage_fraction ≥ 0.95 AND Ra_proxy ≤ 1.6 µm
       │ (OR: any F1–F6 failure → ABORT)
       ▼
┌─────────────┐
│  RETRACT    │  Admittance controller raises TCP 50 mm above
│             │  final contact point; robot returns to home
└──────┬──────┘
       │ Robot at home pose
       ▼
┌─────────────┐
│  VALIDATE   │  Post-process scan: TSDF renders final surface mesh;
│             │  per-zone Ra_proxy computed and logged;
│             │  Metrics written to /metrics/trial_{N}.json
└─────────────┘
```

---

## 4. Quantitative Acceptance Criteria

These criteria define what "the system works" means and bound the thesis evaluation.
All are measured across **N = 50 independent simulated trials** with disturbance parameters
drawn uniformly from the operating envelope defined in Section 2.2.

### 4.1 Primary Criteria (Thesis Pass/Fail)

| Criterion | Symbol | Target | Measurement Method |
|---|---|---|---|
| **Trial success rate** | SR | ≥ 90% | Fraction of N=50 trials completing VALIDATE without F1–F6 |
| **Surface coverage** | SC | ≥ 95% of workpiece area | Area with accumulated material removal h > 0 / total area |
| **Normal force regulation** | F_err | Mean \|F_n - F_setpoint\| < 0.5 N; σ < 0.3 N | Logged at 500 Hz; stats computed over each trial |
| **Surface finish uniformity** | CV(Ra) | Coefficient of variation < 15% across 20 surface zones | Ra_proxy heatmap zoned 20 × (60×40 mm cells) |
| **Replan latency** | t_replan | ≤ 100 ms from drift detection to new waypoints ready | ROS 2 header timestamps on trigger and first new waypoint |
| **TCP tracking error** | e_tcp | RMS < 1.5 mm against current toolpath during 20 mm drift events | Logged at 250 Hz; computed for all drift events |

### 4.2 Secondary Criteria (Thesis Quality / Publication Bar)

| Criterion | Symbol | Target | Notes |
|---|---|---|---|
| **Workpiece pose estimation accuracy** | e_pose | RMS < 0.5 mm position, < 0.5° orientation | Compared to Isaac Sim ground truth |
| **TSDF normal direction accuracy** | e_norm | Mean angular error < 2° vs. ground truth mesh normals | Sampled at 100 waypoint locations per trial |
| **NURBS path smoothness** | κ_max | Max curvature < 5 m⁻¹ anywhere on replanned path | Prevents jerky robot motion |
| **Replanning frequency** | f_replan | Mean 1–5 replans per 120 s trial | Too frequent = overhead issue; too rare = drift accumulates |
| **Cycle time** | T_cycle | Mean < 120 s per 300×200 mm panel | Measured CONTACT_ESTABLISH to RETRACT |
| **ESKF covariance stability** | — | trace(P) remains bounded throughout trial | No divergence events |

### 4.3 Comparative Baseline

Each criterion is compared against two baselines:

- **Baseline A (Open-loop):** Same robot, same nominal toolpath, no closed-loop correction;
  workpiece still drifts. Quantifies the cost of *not* having the system.
- **Baseline B (Offset-only correction):** Same camera, but correction is a rigid-body
  Cartesian offset applied to the whole path at once (Gharaaty et al. 2018 architecture),
  with no mid-pass NURBS replanning. Quantifies the marginal benefit of geometry-aware
  replanning over simple offset compensation.

The primary contribution of the thesis is demonstrating statistically significant improvement
of Proposal A over Baseline B on surface finish uniformity (CV(Ra)) and TCP tracking error
during large displacement events.

---

## 5. System Architecture

### 5.1 Architectural Overview

The system consists of three functional layers with clearly defined interfaces:

```
┌──────────────────────────────────────────────────────────────────┐
│  TASK LAYER                                                      │
│  task_state_machine_node                                         │
│  Orchestrates state transitions; monitors health; logs metrics   │
└────────────────────────────┬─────────────────────────────────────┘
                             │ commands / status
           ┌─────────────────┴──────────────┐
           ▼                                ▼
┌──────────────────────┐        ┌────────────────────────────────┐
│  PERCEPTION LAYER    │        │  PLANNING LAYER                │
│  depth_preprocessor  │        │  toolpath_planner_node         │
│  tsdf_manager_node   │        │  Warm-started NURBS replan     │
│  workpiece_pose_     │        │  on every mesh update          │
│  estimator_node      │        │  (triggered by REPLAN event)   │
│  (DS-ESKF)           │        └───────────────┬────────────────┘
└──────────┬───────────┘                        │ updated waypoints
           │ workpiece pose                     │
           │ @ 1 kHz                            │
           └──────────────────┬─────────────────┘
                              ▼
              ┌───────────────────────────────────┐
              │  MOTION LAYER                     │
              │  motion_executor_node             │
              │  Admittance control               │
              │  Joint velocity commands @ 1 kHz  │
              └───────────────┬───────────────────┘
                              │ joint velocity commands
                              ▼
              ┌───────────────────────────────────┐
              │  SIMULATOR (Isaac Sim)             │
              │  isaac_sim_bridge_node             │
              │  Joint states, F/T, depth image   │
              │  Ground truth poses (evaluation)  │
              └───────────────────────────────────┘
```

Additionally, the `material_removal_proxy_node` sits between the Motion Layer and Perception Layer,
reading TCP pose and F/T to update the TSDF with accumulated material removal at 30 Hz.

### 5.2 ROS 2 Node Graph

Nodes and their primary responsibilities are listed below. All nodes run under ROS 2 Humble
(Ubuntu 22.04). Isaac Sim is connected via the `ros2_bridge` package included with Isaac Sim 4.x.

```
┌──────────────────────────────────────────────────────────────────────┐
│ ISAAC SIM PROCESS                                                    │
│   isaac_sim_bridge_node                                              │
│   Publishes: /camera/depth/image_raw, /camera/color/image_raw,      │
│              /robot/joint_states, /robot/wrench,                     │
│              /ground_truth/workpiece_pose  (for eval only)           │
│   Subscribes: /robot/joint_velocity_cmd                             │
└──────────┬─────────────────────────────────────────────────────────-─┘
           │ sensor streams
┌──────────▼──────────────┐
│ depth_preprocessor_node  │
│ 30 Hz                    │
│ In:  /camera/depth/      │
│       image_raw,         │
│       camera_info        │
│ Out: /workpiece/         │
│      pointcloud_raw      │
└──────────┬───────────────┘
           │ PointCloud2 @ 30 Hz
┌──────────▼───────────────┐         ┌────────────────────────────┐
│ tsdf_manager_node         │◄────────│ material_removal_proxy_node│
│ 30 Hz                     │         │ 30 Hz                      │
│ In:  pointcloud_raw,      │         │ In:  /motion/tcp_pose,     │
│      workpiece_pose       │         │       /robot/wrench        │
│      (for camera pose     │         │ Out: /material_removal/    │
│      compensation),       │         │      tsdf_delta            │
│      material_removal/    │         └────────────────────────────┘
│      tsdf_delta           │
│ Out: /workpiece/mesh,     │
│      /workpiece/normals,  │
│      /workpiece/icp_pose  │
└──────────┬────────────────┘
           │ mesh @ 30 Hz,  icp_pose @ 30 Hz
┌──────────▼───────────────┐
│ workpiece_pose_estimator  │
│ (DS-ESKF)                 │
│ 1 kHz (prediction)        │
│ 30 Hz (update from ICP)   │
│ In:  /workpiece/icp_pose  │
│      /robot/joint_states  │
│ Out: /workpiece/pose_     │
│       estimate (1 kHz)    │
└──────────┬────────────────┘
           │
     ┌─────┴──────────────────────────────────────────┐
     │                                                 │
     │ pose @ 1 kHz                             mesh @ 30 Hz
     ▼                                                 ▼
┌──────────────────────────┐        ┌──────────────────────────────┐
│ motion_executor_node      │◄───────│ toolpath_planner_node        │
│ 250 Hz (Cartesian)        │        │ On-demand (REPLAN trigger)   │
│ 1 kHz (joint velocity)    │ path   │ In:  /workpiece/mesh,        │
│ In:  /toolpath/waypoints, │        │       /toolpath/params        │
│      /workpiece/pose_     │        │ Out: /toolpath/waypoints     │
│       estimate,            │        │ Srv:  /toolpath_planner/     │
│      /robot/wrench,        │        │        replan                │
│      /robot/joint_states   │        └──────────────────────────────┘
│ Out: /robot/joint_        │
│      velocity_cmd,         │
│      /motion/tcp_pose      │
└──────────┬────────────────┘
           │ status, metrics
           ▼
┌──────────────────────────┐
│ task_state_machine_node   │
│ 10 Hz (state management)  │
│ In:  all /task/*, metrics │
│ Out: /task/state,         │
│      /task/metrics        │
│ Srv: /task/start, /abort  │
│ Act: /task/run_task        │
└──────────────────────────┘
```

### 5.3 Topics

All topics use ROS 2 QoS profile `SENSOR_DATA` (best-effort) for high-rate sensor streams and
`RELIABLE` for commands and state transitions.

#### Sensor Input Topics (from Isaac Sim Bridge)

| Topic | Message Type | Rate | Publisher | Description |
|---|---|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | 30 Hz | isaac_sim_bridge | Simulated RealSense D435i depth image |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | isaac_sim_bridge | Intrinsics and distortion |
| `/camera/color/image_raw` | `sensor_msgs/Image` (RGB8) | 30 Hz | isaac_sim_bridge | Color image for texture overlay |
| `/robot/joint_states` | `sensor_msgs/JointState` | 1000 Hz | isaac_sim_bridge | All 6 UR10e joint positions, velocities |
| `/robot/wrench` | `geometry_msgs/WrenchStamped` | 500 Hz | isaac_sim_bridge | Simulated 6-axis F/T sensor at wrist |
| `/ground_truth/workpiece_pose` | `geometry_msgs/PoseStamped` | 100 Hz | isaac_sim_bridge | Isaac Sim ground truth (evaluation only, not used by controller) |

#### Perception Layer Topics

| Topic | Message Type | Rate | Publisher | Description |
|---|---|---|---|---|
| `/workpiece/pointcloud_raw` | `sensor_msgs/PointCloud2` | 30 Hz | depth_preprocessor | Voxel-downsampled point cloud in camera frame |
| `/workpiece/mesh` | `shape_msgs/Mesh` | 30 Hz | tsdf_manager | Current marching-cubes mesh of workpiece surface |
| `/workpiece/normals` | `geometry_msgs/PoseArray` | 30 Hz | tsdf_manager | Per-vertex surface normals (position = vertex, orientation = normal) |
| `/workpiece/icp_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 30 Hz | tsdf_manager | ICP-estimated workpiece pose + covariance |
| `/workpiece/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | 1000 Hz | workpiece_pose_estimator | DS-ESKF fused workpiece pose |

#### Planning Layer Topics

| Topic | Message Type | Rate | Publisher | Description |
|---|---|---|---|---|
| `/toolpath/waypoints` | `nav_msgs/Path` | On update | toolpath_planner | Ordered list of SE(3) waypoints in `workpiece_live_frame` |
| `/toolpath/current_waypoint_idx` | `std_msgs/Int32` | 250 Hz | motion_executor | Index of waypoint currently being tracked |
| `/toolpath/replan_trigger` | `std_msgs/Header` | Event | workpiece_pose_estimator | Emitted when drift > 2 mm threshold; triggers replan |
| `/toolpath/params` | `custom_msgs/ToolpathParams` | On change | task_state_machine | Sanding speed, normal force setpoint, grit pitch |

#### Motion Layer Topics

| Topic | Message Type | Rate | Publisher | Description |
|---|---|---|---|---|
| `/robot/joint_velocity_cmd` | `std_msgs/Float64MultiArray` | 1000 Hz | motion_executor | 6-DOF joint velocity commands (rad/s) |
| `/motion/tcp_pose` | `geometry_msgs/PoseStamped` | 1000 Hz | motion_executor | FK-computed TCP pose in world frame |
| `/motion/force_error` | `std_msgs/Float64` | 500 Hz | motion_executor | Signed F_n error (N), for monitoring |

#### Task and Metrics Topics

| Topic | Message Type | Rate | Publisher | Description |
|---|---|---|---|---|
| `/task/state` | `std_msgs/String` | 10 Hz | task_state_machine | Current state machine state |
| `/task/metrics` | `custom_msgs/TaskMetrics` | 1 Hz | task_state_machine | Aggregated trial metrics (coverage, Ra proxy, force stats, timing) |
| `/material_removal/roughness_map` | `sensor_msgs/Image` | 5 Hz | material_removal_proxy | Per-zone Ra proxy heatmap (grayscale, zones as pixels) |
| `/material_removal/coverage_fraction` | `std_msgs/Float64` | 1 Hz | material_removal_proxy | Fraction of surface with h_accumulated > 0 |
| `/material_removal/tsdf_delta` | `custom_msgs/TSDFDelta` | 30 Hz | material_removal_proxy | Voxel-wise material removal increment for TSDF |

### 5.4 Services

| Service | Type | Server | Description |
|---|---|---|---|
| `/tsdf_manager/reset` | `std_srvs/Trigger` | tsdf_manager | Clear TSDF volume and re-initialise for new trial |
| `/tsdf_manager/get_mesh_snapshot` | `custom_srvs/GetMesh` | tsdf_manager | Returns current mesh as `shape_msgs/Mesh` synchronously |
| `/toolpath_planner/replan` | `custom_srvs/Replan` | toolpath_planner | Request: mesh + current waypoint index. Response: updated waypoints. Synchronous, max 100 ms. |
| `/toolpath_planner/set_params` | `rcl_interfaces/SetParameters` | toolpath_planner | Set grit pitch, overlap fraction, speed, force setpoint |
| `/motion_executor/set_force_setpoint` | `custom_srvs/SetForce` | motion_executor | Update F_n target mid-task |
| `/motion_executor/pause` | `std_srvs/Trigger` | motion_executor | Pause joint velocity output (safety stop) |
| `/motion_executor/resume` | `std_srvs/Trigger` | motion_executor | Resume after pause |
| `/task_state_machine/abort` | `std_srvs/Trigger` | task_state_machine | Immediately trigger RETRACT regardless of current state |
| `/task_state_machine/get_state` | `custom_srvs/GetState` | task_state_machine | Synchronously return current state enum |

**Custom service definitions:**

```
# custom_srvs/Replan.srv
shape_msgs/Mesh current_mesh
int32 current_waypoint_idx
---
nav_msgs/Path updated_waypoints
bool success
string message
float64 solve_time_ms
```

```
# custom_srvs/GetMesh.srv
---
shape_msgs/Mesh mesh
std_msgs/Header stamp
```

### 5.5 Actions

| Action | Type | Server | Description |
|---|---|---|---|
| `/perception/initial_scan` | `custom_actions/InitialScan` | tsdf_manager | Acquire N frames, fuse TSDF, return initial mesh + workpiece_nominal_frame pose. Feedback: frame count + mesh quality metric. |
| `/motion_executor/execute_toolpath` | `custom_actions/ExecuteToolpath` | motion_executor | Execute current toolpath with online admittance control and replan integration. Feedback: waypoint index, tracking error, force error, coverage. Result: success/failure + final metrics. |
| `/task_state_machine/run_task` | `custom_actions/RunTask` | task_state_machine | Top-level task execution from APPROACH through VALIDATE. Feedback: current state + task metrics. Result: pass/fail + full metrics log path. |

**Custom action definitions:**

```
# custom_actions/InitialScan.action
int32 num_frames          # How many frames to fuse (default: 5)
---
shape_msgs/Mesh initial_mesh
geometry_msgs/Pose workpiece_nominal_pose
float64 registration_fitness
bool success
---
int32 frames_acquired
float64 tsdf_voxel_count
```

```
# custom_actions/RunTask.action
string trial_id
float64 drift_magnitude_mm    # Injected disturbance amplitude for this trial
---
bool success
string failure_code           # empty if success, else F1–F6
float64 coverage_fraction
float64 ra_proxy_mean
float64 ra_proxy_cv
float64 cycle_time_s
float64 mean_force_error_n
float64 rms_tcp_error_mm
int32 replan_count
---
string current_state
float64 coverage_fraction
float64 mean_force_error_n
float64 rms_tcp_error_mm
```

### 5.6 TF Tree

All transforms are published via `tf2_ros`. Static transforms are published by
`robot_state_publisher` and the camera mount node. Dynamic transforms are
published at the rates indicated.

```
world (fixed)
├── table_frame (static) — coincides with world; table surface at Z = 0
│
├── camera_mount_frame (static) — camera rigidly bolted above table at
│   │                             world position [0.5, 0.0, 1.2] m, tilted 30°
│   └── camera_depth_optical_frame (static) — optical axis of depth sensor
│       └── camera_color_optical_frame (static) — optical axis of color sensor
│
├── robot_base_link (static) — UR10e base, bolted to world at [-0.3, 0.0, 0.0]
│   └── robot_shoulder_link
│       └── robot_upper_arm_link
│           └── robot_forearm_link
│               └── robot_wrist_1_link
│                   └── robot_wrist_2_link
│                       └── robot_wrist_3_link
│                           └── tool_flange (published by robot_state_publisher @ 1 kHz)
│                               └── tcp_frame — sander contact centroid
│                                   └── abrasive_disc_frame — disc plane origin
│
├── workpiece_nominal_frame (static after INIT_SCAN)
│   — Set once from ICP registration at initialization
│   — Never updated; serves as reference for baseline comparison
│   — Parent: world
│
└── workpiece_live_frame (dynamic, published @ 1 kHz by DS-ESKF)
    — Continuously updated workpiece pose from DS-ESKF
    — Parent: world
    — This is the reference frame for all toolpath waypoints
    └── workpiece_surface_frame (static relative to workpiece_live_frame)
        — Mesh origin (centroid of workpiece top face)
        — Z-axis points outward from sanded surface
```

**Key TF lookup used by motion_executor:**
At each 250 Hz control cycle, the motion executor looks up:
```
world → workpiece_live_frame → toolpath waypoint_i
world → tcp_frame
```
to compute the tracking error and the desired TCP pose for the admittance controller.

### 5.7 Node-by-Node Descriptions

#### `isaac_sim_bridge_node`

Wraps the Isaac Sim `ros2_bridge` extension. Configured via `isaac_sim_bridge.yaml` to spawn
the UR10e URDF, a RealSense D435i camera actor, the aluminum panel rigid body with
PTFE-friction material properties, and the disturbance injector (a scripted force on the
panel's center of mass). Publishes all sensor topics; subscribes to joint velocity commands
and passes them to Isaac Sim's articulation controller.

The disturbance injector publishes a `geometry_msgs/WrenchStamped` to `/sim/disturbance_force`
(for logging only) and directly applies the force in the physics engine via the
`omni.isaac.core.articulations` API.

#### `depth_preprocessor_node`

Converts raw depth images to point clouds:
1. Apply depth threshold: keep only points in [0.15 m, 0.8 m] from camera (workpiece range).
2. Project to 3D using `camera_info` intrinsics.
3. Voxel-downsample to 3 mm grid (Open3D `voxel_down_sample`).
4. Remove outliers (radius outlier removal, radius = 10 mm, min_points = 20).
5. Publish `sensor_msgs/PointCloud2` in `camera_depth_optical_frame`.

Latency target: < 10 ms per frame.

#### `tsdf_manager_node`

Maintains the TSDF volume (Open3D `ScalableTSDFVolume`, voxel_length = 1 mm, sdf_trunc = 3 mm).

On each 30 Hz depth frame:
1. Look up `world → camera_depth_optical_frame` from TF.
2. Look up `world → workpiece_live_frame` (previous ESKF estimate) to set the TSDF origin
   and ensure the integration is in a workpiece-fixed reference.
3. Call `tsdf.integrate(rgbd_image, intrinsic, extrinsic)` where extrinsic is the camera
   pose in the workpiece frame.
4. Also subtract material removal increments from `tsdf_delta` (see material_removal proxy).
5. Run marching cubes (`tsdf.extract_triangle_mesh()`) to get a mesh.
6. Compute per-vertex normals.
7. Run 2D ICP between current point cloud and the previous mesh to get updated workpiece pose:
   - Open3D `registration_icp` with `point-to-plane` metric, max_correspondence_distance = 5 mm.
   - Fitness threshold: > 0.6; if below, keep previous estimate and flag F5 condition.
8. Publish `/workpiece/mesh`, `/workpiece/normals`, `/workpiece/icp_pose`.

The TSDF also tracks a `coverage_mask`: a 2D binary array (300×200 cells, 1 mm resolution)
indicating which XY locations have had material removed, published into the material_removal
proxy's roughness map.

#### `workpiece_pose_estimator_node` (DS-ESKF)

Implements the Dual-Space Error-State Kalman Filter from Wen & Pagilla (2023), extended to fuse
a third modality (F/T sensor) as a weak motion prior.

**State vector** (16-dimensional):
- Position p ∈ ℝ³ (workpiece centroid in world frame)
- Quaternion q ∈ ℍ (workpiece orientation; stored as unit quaternion, error state in so(3))
- Linear velocity v ∈ ℝ³
- Angular velocity ω ∈ ℝ³

**Prediction (1 kHz):** Uses forward kinematics on joint states to compute TCP velocity,
then estimates the workpiece velocity via a contact reaction model:
`v_workpiece ≈ α × (F_lateral / m_workpiece × dt)` where α ≈ 0.1 is a coupling coefficient
(very weak coupling; mostly relies on state propagation).

**Measurement update (30 Hz):** ICP pose estimate from tsdf_manager_node.
Observation model: H = I₆ (identity on [p, q_euler]).
Innovation covariance R_icp = diag([0.5² mm², 0.5² mm², 0.5² mm², 0.5°², 0.5°², 0.5°²]).

Publishes `workpiece_live_frame` to TF at 1 kHz, and `/workpiece/pose_estimate`
with full covariance matrix.

Also monitors ESKF drift: if `e_pose > 2 mm` (ICP measurement vs. propagated prediction),
publishes `/toolpath/replan_trigger`.

#### `toolpath_planner_node`

Generates and re-generates the sanding toolpath from the current mesh.

**On startup (initial plan):**
1. Project mesh vertices onto the XY plane.
2. Generate a raster pattern: parallel scan lines spaced 40 mm (50% overlap of 80 mm disc),
   oriented along the panel's long axis (X direction), with alternating left-right directions.
3. For each raster point, find the nearest mesh vertex and its normal.
4. Construct a SE(3) waypoint: position = mesh vertex (offset 5 mm above surface in normal direction),
   orientation = Z-axis aligned to surface normal, X-axis aligned to scan direction.
5. Fit dual NURBS: P(t) through all 3D waypoint positions, N(t) through the corresponding
   normal-aligned quaternions.
   - Degree 5 B-spline, Clamped knot vector.
   - Smoothing: minimize `∫₀¹ ||P''(t)||² dt` subject to interpolation constraints.
     This is a banded linear system solved via `scipy.linalg.solve_banded` in ~5 ms.
6. Stiffness optimization: At each waypoint, compute the robot's Cartesian stiffness ellipsoid
   (from Jacobian and joint stiffness model); choose the wrist redundancy angle to maximize
   stiffness in the surface-normal direction.
7. Publish to `/toolpath/waypoints`.

**On REPLAN trigger:**
1. Check `/task/state` is SANDING.
2. Call `/tsdf_manager/get_mesh_snapshot`.
3. Re-run steps 3–6 above on the updated mesh.
4. Warm start: initialize NURBS control points from the previous solution; only update
   control points in the neighborhood of vertices where the pose has changed by > 1 mm.
5. Publish updated `/toolpath/waypoints` within 100 ms.

**Parameters** (settable via `/toolpath_planner/set_params`):
- `raster_spacing_mm`: 40.0
- `surface_offset_mm`: 5.0
- `tcp_speed_mm_s`: 50.0
- `nurbs_degree`: 5
- `smoothing_weight`: 1.0

#### `motion_executor_node`

Implements the admittance control law and joint velocity resolver.

**Admittance controller (250 Hz Cartesian loop):**

Force error in surface-normal direction:
```
Δf_n = F_n_setpoint − F_n_measured
```

Normal-direction admittance acceleration:
```
ẍ_n = Δf_n / M_d − B_d/M_d × ẋ_n
```
with virtual mass `M_d = 0.5 kg`, virtual damping `B_d = 50 N·s/m`.

Lateral position error (in workpiece surface XY plane):
```
Δp_lateral = p_waypoint(t) − p_tcp_projected(t)
```

Combined Cartesian velocity command:
```
v_cmd = K_p × Δp_lateral + v_waypoint_feedforward
        + ẋ_n × n̂_surface
```
where `K_p = 2.0` and `n̂_surface` is the surface normal at the current waypoint.

Orientation error: quaternion error between TCP orientation and waypoint orientation,
converted to angular velocity with gain `K_ω = 3.0`.

**Joint velocity resolver (1 kHz):**

Resolved-rate control via damped least-squares Jacobian pseudoinverse:
```
q̇ = Jᵀ(JJᵀ + λI)⁻¹ × v_cmd_cartesian
```
with `λ = 0.01` (damping), computed from the UR10e analytical Jacobian.

Joint velocity limits enforced: `|q̇_i| ≤ q̇_max_i` (UR10e specs).
Publishes `/robot/joint_velocity_cmd` at 1 kHz via the Isaac Sim bridge.

**Waypoint advancement logic:**
- Advance to next waypoint when: lateral error `< 2 mm` AND has been tracking for `≥ 0.5 s`.
- On REPLAN: find the closest waypoint on the new path to the current TCP position;
  resume from there immediately without stopping.

#### `material_removal_proxy_node`

Implements a Preston-law material removal model to track surface finish evolution.

**Preston's law:**
```
dh/dt = k_p × P × v_rel
```
where:
- `k_p = 2.5 × 10⁻¹² Pa⁻¹` (Preston constant for Al 6061 with P120 grit)
- `P = F_n / A_contact` (contact pressure; `A_contact = π × (40 mm)² ≈ 5026 mm²`)
- `v_rel = ||v_tcp||` (relative velocity between tool and surface)

At 30 Hz, for the contact region (voxels beneath the TCP footprint):
1. Compute `Δh = k_p × P × v_rel × Δt` (accumulated removal this cycle).
2. Add `Δh` to the per-voxel `h_accumulated` array.
3. Compute `Ra_proxy_i = Ra_initial × exp(−γ × h_accumulated_i)` with `γ = 0.8 µm⁻¹`
   (a monotonically decreasing model of surface roughness with material removed).
4. Publish `Ra_proxy` as a heatmap image and the scalar coverage fraction.
5. Publish `TSDFDelta` (updated voxel values) to tsdf_manager.

The `VALIDATE` state computes final per-zone statistics from the accumulated `h` array.

#### `task_state_machine_node`

A finite state machine (Python `transitions` library) implementing the process flow from
Section 3. Monitors all health conditions (F1–F6) by subscribing to `/robot/wrench`,
`/workpiece/pose_estimate`, `/motion/tcp_pose`, and `/material_removal/coverage_fraction`.

Logs all trial metrics to `~/.ros/metrics/trial_{trial_id}.json` at VALIDATE completion.

### 5.8 Interface Contracts Between Subsystems

These are the formal contracts that define the boundaries between subsystems. If any node
is replaced (e.g., swapping the ESKF for a different estimator), only the contract matters.

| Interface | From | To | Contract |
|---|---|---|---|
| A: ICP pose | tsdf_manager | workpiece_pose_estimator | `PoseWithCovarianceStamped` in `world` frame, ICP fitness ≥ 0.6, latency ≤ 33 ms |
| B: Workpiece pose | workpiece_pose_estimator | motion_executor, toolpath_planner | `PoseWithCovarianceStamped` at 1 kHz, position σ ≤ 2 mm, orientation σ ≤ 1° |
| C: Mesh | tsdf_manager | toolpath_planner | `shape_msgs/Mesh`, normals computed, max 50k triangles, latency ≤ 33 ms |
| D: Waypoints | toolpath_planner | motion_executor | `nav_msgs/Path` in `workpiece_live_frame`, waypoints ≥ 5 mm apart, curvature < 5 m⁻¹ |
| E: Joint velocity | motion_executor | isaac_sim_bridge | `Float64MultiArray`, 6 elements, rate 1 kHz, within UR10e velocity limits |
| F: TSDF delta | material_removal_proxy | tsdf_manager | `custom_msgs/TSDFDelta`, list of (voxel_index, delta_h_m) pairs |
| G: Replan trigger | workpiece_pose_estimator | toolpath_planner | `std_msgs/Header` (timestamp of trigger); toolpath_planner must respond with updated waypoints within 100 ms |

---

## 6. Algorithm Specifications

### 6.1 TSDF Volume Management

The TSDF volume is the system's single source of truth about workpiece geometry.

**Volume parameters:**
- Voxel length: 1.0 mm (allows 1 mm surface reconstruction resolution)
- SDF truncation: 3.0 mm (3× voxel length, standard)
- Volume size: scalable (Open3D `ScalableTSDFVolume` with block resolution 16)
- Memory estimate: ~300 MB for the 300×200×30 mm active region

**Integration equation** (per frame):
```
TSDF(x) ← (w(x) × TSDF(x) + w_new × SDF_new(x)) / (w(x) + w_new)
w(x) ← w(x) + w_new
```
where `w_new = 1` for voxels within the camera frustum and `SDF_new(x)` is the signed
distance from the measured surface.

**Material removal integration:**
Material removal proxy provides `Δh` per voxel per timestep (in meters). The TSDF manager
updates:
```
TSDF(x) ← TSDF(x) − Δh(x) / sdf_trunc   (when Δh(x) > 0)
```
This shifts the zero-crossing outward (toward the tool), simulating surface recession.

**Normal estimation:** Via `compute_vertex_normals()` on the extracted mesh, using the
angle-weighted average of adjacent face normals.

### 6.2 Dual-Space Error-State Kalman Filter (DS-ESKF)

Wen & Pagilla (2023) formulation, adapted for workpiece tracking (not robot TCP tracking).

**Nominal state:** `X = [p, q, v, ω]` where `q` is a unit quaternion.
**Error state:** `δX = [δp, δθ, δv, δω]` (all in ℝ³, with δθ ∈ so(3)).

**Prediction at 1 kHz (between ICP updates):**

Workpiece dynamics model (quasi-static rigid body on low-friction surface):
```
p_{k+1} = p_k + v_k × Δt
q_{k+1} = q_k ⊗ exp(ω_k × Δt / 2)
v_{k+1} = v_k + a_disturbance × Δt − µ_kinetic × g × sign(v_k) × Δt
ω_{k+1} = ω_k − µ_rot × ω_k × Δt
```
where `µ_kinetic = 0.02` (PTFE table), `µ_rot = 2.0 rad⁻¹s⁻¹` (rotational damping).

Process noise covariance Q (tuned empirically in simulation):
```
Q = diag([σ_p², σ_p², σ_p², σ_θ², σ_θ², σ_θ², σ_v², σ_v², σ_v², σ_ω², σ_ω², σ_ω²])
  = diag([0.1² mm², ..., 0.05° ², ..., 5² mm²/s², ..., 2° ²/s², ...])
```

**Measurement update at 30 Hz (from ICP):**

Observation model: H = I₁₂ restricted to [p, q_euler].
```
K = P × Hᵀ × (H × P × Hᵀ + R_ICP)⁻¹
δX ← K × (z_ICP − h(X))
X ← X ⊕ δX    (error-state injection on manifold)
P ← (I − K × H) × P
```

R_ICP is scaled by ICP fitness: `R_ICP = R_base / fitness²`, so a poor ICP result
(low fitness) inflates the noise and trusts the prediction more.

**Drift detection:**
```
if ||p_ICP − p_predicted|| > 2 mm OR ||q_ICP − q_predicted||_angle > 1°:
    publish /toolpath/replan_trigger
```

### 6.3 NURBS Toolpath Planner

**Data structures:**
- `P_ctrl[n_ctrl][3]`: n_ctrl × 3 array of NURBS position control points
- `N_ctrl[n_ctrl][4]`: n_ctrl × 4 array of NURBS orientation control points (quaternions)
- `knots[n_ctrl + degree + 1]`: clamped uniform knot vector

**Smoothing objective** (position path only; orientation path uses SLERP):
```
min   λ_smooth × ∫₀¹ ||P''(t)||² dt  +  λ_data × Σᵢ ||P(tᵢ) − dᵢ||²
```
where `dᵢ` are the target waypoint positions and `tᵢ` are the corresponding parameter values.

Discretizing with m = 200 integration points:
```
min  x^T Q_smooth x  +  λ × ||A x − d||²
```
where `x = P_ctrl.flatten()`, `Q_smooth` is the second-derivative Gram matrix (banded),
`A` is the NURBS evaluation matrix at waypoint parameters.

This is a banded symmetric positive semi-definite system, solved in ~5 ms.

**Warm start on replan:**
```
x_init = previous P_ctrl
only re-optimize control points where ||d_new_i − d_old_i|| > 1 mm
```
This reduces the active set from ~200 to ~20 points on small drifts, giving ~0.5 ms
solve time for typical drift events.

**Stiffness optimization** (per waypoint):
For each waypoint, the UR10e's 7th DOF (wrist-3 joint angle) is swept over [−π, π] in 36
steps. At each, the manipulability ellipsoid and Cartesian stiffness `K_c = J⁻ᵀ K_j J⁻¹`
are computed (where `K_j = diag([5000, 5000, 5000, 5000, 5000, 5000] N/rad) is the joint
stiffness model`). The wrist angle maximizing `n̂ᵀ K_c n̂` (stiffness in the surface-normal
direction) is selected. This adds ~15 ms to the full initial plan; warm-start replans skip
waypoints where the normal has not changed.

### 6.4 Admittance Controller

The admittance controller outputs Cartesian velocity commands at 250 Hz to the
joint velocity resolver running at 1 kHz.

**Force channel (normal direction):**
```
e_f(t) = F_setpoint − F_n(t)         [N]
ẍ_n(t) = e_f / M_d − B_d × ẋ_n / M_d
ẋ_n(t+dt) = ẋ_n(t) + ẍ_n(t) × dt
```
Anti-windup: `ẋ_n` clamped to `[−5, +5]` mm/s.

**Lateral position channel:**
```
e_p(t) = p_waypoint(t) − p_tcp_surface_projection(t)
v_lateral(t) = K_p × e_p(t) + v_feedforward(t)
```
`v_feedforward = 50 mm/s` in the current raster direction.

**Orientation channel:**
```
e_q = q_waypoint ⊗ q_tcp⁻¹
e_ω = 2 × vec(e_q)   [axis-angle approximation for small errors]
v_angular = K_ω × e_ω    [rad/s]
```

**Combined 6D velocity command:**
```
V_cmd = [v_lateral + ẋ_n × n̂,  v_angular]   [m/s, rad/s]
```

**Joint velocity resolver:**
```
J = UR10e_jacobian(q_current)   [6×6 analytical Jacobian]
λ = 0.01 if det(JJᵀ) > 1e-4 else 0.1   [adaptive damping near singularity]
q̇ = Jᵀ(JJᵀ + λI)⁻¹ × V_cmd
q̇ = clip(q̇, -q̇_max, q̇_max)
```

### 6.5 Preston-Law Material Removal Proxy

```python
# Called at 30 Hz
def update_material_removal(F_n, v_tcp, tcp_pose, dt=1/30):
    # Contact pressure
    A_contact = np.pi * (0.04)**2   # 80 mm disc radius
    P = F_n / A_contact              # [Pa]

    # Relative velocity
    v_rel = np.linalg.norm(v_tcp)   # [m/s]

    # Material removal depth this timestep
    delta_h = K_PRESTON * P * v_rel * dt   # [m]

    # Find voxels in contact footprint
    footprint_voxels = tsdf.query_footprint(tcp_pose, radius=0.04)

    # Update accumulation array
    h_accumulated[footprint_voxels] += delta_h

    # Ra proxy: exponential decay model
    Ra_proxy[footprint_voxels] = (
        RA_INITIAL * np.exp(-GAMMA * h_accumulated[footprint_voxels])
    )

    # Coverage
    coverage = np.mean(h_accumulated > 0)
```

**Constants:**
- `K_PRESTON = 2.5e-12` Pa⁻¹ (empirical for Al 6061 + P120 grit)
- `RA_INITIAL = 3.2e-6` m (Ra of unmachined panel)
- `GAMMA = 0.8e6` m⁻¹ (exponential roughness decay constant; yields Ra = 1.6 µm at `h = 0.9 µm`)

---

## 7. Implementation Milestones

The thesis is structured as five phases over 18 months. The first phase de-risks all
downstream integration by building on a static, fixtured workpiece.

### Phase 1 — Static Baseline (Months 1–3)

**Goal:** End-to-end sanding pipeline working on a fixtured, non-drifting workpiece.
No closed-loop correction; validates every component in isolation.

| Deliverable | Description |
|---|---|
| Isaac Sim scene | UR10e + camera + aluminum panel; UR10e ROS 2 driver active |
| depth_preprocessor | Point cloud from depth image, validated against ground truth |
| tsdf_manager | Static workpiece TSDF, mesh extraction, normal accuracy < 2° |
| toolpath_planner | Initial NURBS toolpath on mesh; visualized in RViz |
| motion_executor | Open-loop trajectory execution; robot tracks path without force control |
| material_removal_proxy | Preston-law accumulation; coverage map verified against known path |

**Acceptance check:** Robot traces nominal raster path on fixtured workpiece; coverage > 95%;
force sensor reads approximately the target F_n (no closed-loop yet, just passive contact).

### Phase 2 — Force Control and Contact Stability (Months 3–5)

**Goal:** Admittance controller maintains F_n = 8 N ± 1 N over a full sanding pass on a
static fixtured workpiece.

| Deliverable | Description |
|---|---|
| Admittance controller | Force channel active; anti-windup; singularity handling |
| Joint velocity resolver | Damped least-squares; joint velocity limits |
| F/T sensor simulation | Noise model tuned to match real RealSense + ATI Mini45 specs |
| Metrics logging | Force error statistics computed and logged per trial |

**Acceptance check:** Mean `|F_n − F_setpoint| < 0.5 N`, σ `< 0.3 N` over 10 trials on static part.

### Phase 3 — Perception and Pose Estimation (Months 5–9)

**Goal:** DS-ESKF tracks a manually displaced workpiece (discrete displacements injected by teleop)
with pose error < 0.5 mm.

| Deliverable | Description |
|---|---|
| ESKF implementation | Prediction + ICP update + drift detection |
| ICP integration | Open3D point-to-plane ICP; fitness thresholding |
| TF broadcasting | workpiece_live_frame published at 1 kHz |
| Occlusion analysis | Quantify % of surface visible as a function of camera position |
| ESKF validation | Compare ESKF pose to Isaac Sim ground truth; RMS < 0.5 mm |

**Acceptance check:** After a 20 mm XY discrete step displacement, ESKF converges to within
0.5 mm of ground truth within 5 ICP frames (< 167 ms).

### Phase 4 — Closed-Loop Integration (Months 9–14)

**Goal:** Full closed-loop system: continuous drift + ESKF + replan + admittance control operating
simultaneously.

| Deliverable | Description |
|---|---|
| Replan trigger integration | ESKF drift detection → toolpath_planner service call |
| Warm-start replanning | NURBS warm-start; replan latency ≤ 100 ms for typical drifts |
| Seamless waypoint handoff | motion_executor resumes from nearest waypoint on new path without pausing |
| Disturbance injection | Full scripted disturbance model active (quasi-static + impulse + rotation) |
| N=50 trial evaluation | All primary and secondary acceptance criteria evaluated |
| Baseline A + B comparison | Open-loop baseline and offset-only baseline implemented and evaluated |

**Acceptance check:** All primary criteria from Section 4.1 satisfied at N=50. Statistical
significance (p < 0.05, Wilcoxon rank-sum) of improvement over Baseline B on CV(Ra).

### Phase 5 — Thesis Write-up and Ablation Studies (Months 14–18)

**Goal:** Characterize the contribution of each subsystem and write the thesis.

| Deliverable | Description |
|---|---|
| Ablation 1: NURBS vs offset correction | Fix everything else; swap replanning strategy |
| Ablation 2: Multi-rate vs single-rate | DS-ESKF vs ICP-only at 30 Hz |
| Ablation 3: Warm-start vs cold-start | Quantify replanning latency improvement |
| Sensitivity sweep | Camera frame rate: 10 / 30 / 60 / 120 Hz; plot SR, CV(Ra) vs frame rate |
| Thesis draft | Chapters: Introduction, Background, System Design, Results, Discussion |
| Conference paper | Target ICRA or IROS (8-page version of Phase 4 results) |

---

## 8. Risk Register and Mitigation

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| **R1: Contact dynamics fidelity** — PhysX friction model does not reproduce real PTFE-surface drift | High | High | Validate friction model against published tribology data; report sensitivity to friction coefficient; explicitly scope contribution as "simulation study" |
| **R2: TSDF update during tool occlusion** — sander disc blocks camera view of contact region | High | Medium | Multi-camera setup (2 cameras at different angles, trivial in simulation); inpaint occluded region using adjacent TSDF values |
| **R3: ICP failure under dust/motion blur** — depth image noisy during sanding | Medium | Medium | Depth noise model tuned to worst-case RealSense spec; robust point-to-plane ICP with adaptive correspondence threshold; ESKF handles intermittent ICP failures gracefully |
| **R4: Replan latency exceeds 100 ms** — NURBS solver too slow for large drifts | Medium | Medium | Warm-start reduces active control points; if needed, reduce NURBS degree from 5 to 3; as a last resort, accept 200 ms and update acceptance criterion with rationale |
| **R5: ESKF divergence under large impulse** — sudden 20 mm displacement violates filter assumptions | Medium | High | Inflate process noise Q during impulse (detected via anomalously large ICP innovation); re-initialize ESKF from ICP if innovation > 10 mm |
| **R6: Isaac Sim version instability** — APIs change between versions | Low | High | Pin Isaac Sim to 4.2; document exact version; test upgrade path to 4.x |
| **R7: Scope creep** — additional experiments delay thesis completion | Medium | Medium | Phase gate approach: do not start Phase 5 ablations until all primary acceptance criteria confirmed |

---

## 9. Thesis Contribution Statement

The thesis will claim the following original contributions, each defensible against the state of the art surveyed:

**C1 (Primary):** The first published demonstration, in simulation, of uninterrupted
geometry-aware toolpath replanning on a free-floating workpiece during continuous robotic
abrasive contact. This distinguishes the work from: Wang et al. (2022) (geometry-aware,
but single-shot offline); Tian et al. (2023) (geometry-aware, but between passes);
Gharaaty et al. (2018) (in-pass correction, but rigid-body offset only, not geometry-aware).

**C2:** A multi-rate fusion architecture (DS-ESKF with ICP + encoder + F/T modalities) that
maintains workpiece pose estimation during contact-induced TSDF occlusion, extending Wen &
Pagilla (2023) (vision + proprioception only) with a force-sensor innovation term.

**C3:** A warm-started NURBS replanning strategy that reduces mid-pass replanning latency
from O(full solve) to O(local update) by identifying and re-optimizing only the
geometrically affected segments of the toolpath.

**C4 (Applied):** A quantified sensitivity analysis of closed-loop sanding performance as
a function of camera frame rate, establishing the minimum perception bandwidth required to
maintain contact-finish quality at given drift rates — a design parameter currently absent
from the manufacturing literature.

---

*Plan version: 1.0 | Prepared for GrayMatter Robotics Master's Thesis Advising*
*Literature sources: Wang et al. 2022 (IEEE T-RO); Wen & Pagilla 2023 (RAS); Gharaaty et al. 2018 (IJARS);
Tian et al. 2023 (JMP); Ginhoux et al. 2004 (ICRA); Liu et al. 2021 (IROS); Zhu et al. 2020 (RCIM)*
