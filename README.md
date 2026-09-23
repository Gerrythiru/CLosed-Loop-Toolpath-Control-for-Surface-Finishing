# Closed-Loop Toolpath Control for Surface Finishing

A MuJoCo simulation of a robot arm that **keeps its surface-finishing toolpath on
target while the workpiece moves under it**. A UFactory Lite 6 arm rasters a tool
across a 30 × 30 cm aluminum plate. The plate is not clamped and drifts
unpredictably during the pass. An emulated OptiTrack motion-capture rig tracks
the plate, and the planned path is re-projected through that live pose estimate
on every control tick.

<p align="center">
  <img src="toolpath_trace.gif" width="560" alt="Lite 6 arm tracing a serpentine toolpath across the plate">
  <br><em>The arm tracing an ordered toolpath on the plate. The red trail is the executed tool-tip path.</em>
</p>

## Why

Robotic sanding, polishing and deburring usually assume the part is rigidly
fixtured, so a pre-planned path is correct for the whole pass. When the part
shifts because of loose fixturing, compliant supports or contact forces, an
open-loop path drifts off the surface it was meant to cover. This project builds
the full sense → estimate → correct loop in simulation and measures how much of
that error closing the loop removes. It uses realistic sensor imperfections,
including noise, latency, occlusion, marker swaps and ghost detections.


## How it works

```mermaid
flowchart LR
    D["Plate disturbance<br/>stepped ±1.5° yaw, PD torque<br/>on a real 3 kg free body"] --> P(("Plate<br/>true pose"))
    P --> M["Emulated mocap rig<br/>3 × PrimeX 22, 4 markers, 240 Hz<br/>noise · latency · occlusion · swaps · ghosts"]
    M --> F["Pose estimate<br/>triangulation + robust rigid fit<br/>outlier rejection + temporal gate"]
    F --> C["Consumer guards<br/>hold through dropout<br/>rate limiter"]
    C --> R["Re-project path<br/>plate-local plan → world"]
    R --> K["6-DOF DLS IK<br/>+ look-ahead feed-forward"]
    K --> A["Lite 6 position actuators"]
    A -->|tool contact| P
```

The project was built in phases, and each phase has its own verification script:

| Phase | What | Key script |
|---|---|---|
| Scene | Table, free-body plate, Lite 6 with probe tool; full-plate reach verified | `check_reach.py` |
| Toolpath | Constant-feedrate serpentine raster, IK + feed-forward tracking | `trace_toolpath.py` |
| Mocap rig | 3 cameras placed outside the robot's swept volume and keep-out zone | `check_mocap_rig.py` |
| D: sensing | Noise, occlusion, bias/drift, label swaps, ghosts → robust pose fit | `mocap_emulator.py` |
| E: disturbance | Scripted stepped-yaw motion of the plate | `plate_disturbance.py` |
| F: closed loop | Path re-projected through the live estimate; open vs. closed vs. oracle | `closed_loop_demo.py` |

## Results

Each mode runs the same 23 s toolpath under the same disturbance (seed 2). The
error is measured against the plate's **true** pose, not the estimate:

| Mode | What drives the path | Mean contact error | Max |
|---|---|---|---|
| Open loop | Nominal plate pose, no correction | 5.39 mm | 14.5 mm |
| **Closed loop** | **Live mocap estimate** | **2.94 mm** | 19.2 mm* |
| Oracle | Perfect, zero-latency ground truth | 2.80 mm | 7.2 mm |

Closing the loop **cuts mean contact error by about 45%**, to within about 0.15 mm
of the perfect-sensor oracle. The error that remains, roughly 2.8 mm, matches the
arm's own tracking lag on a *static* plate. That makes it the arm's precision
limit, not a sensing problem.

\*The closed-loop maximum comes from one short transient near t ≈ 21.7 s, when a
bad pose estimate briefly got past the filters (visible in both panels below).
The other maxima in this mode stay at the oracle level.

<p align="center">
  <img src="closed_loop_result.png" width="780" alt="Contact error and plate yaw for open, closed and oracle modes">
  <br><em>Top: tool-to-plate contact error for each mode. Bottom: true plate yaw vs. the held mocap estimate.</em>
</p>

With a different disturbance schedule (seed 7), the plate drifts about 7° in one
direction. Open-loop error grows past 20 mm, while closed-loop stays with the oracle:

<p align="center">
  <img src="closed_loop_result_Sd7.png" width="780" alt="Closed-loop result with disturbance seed 7">
</p>

## Gallery

<table>
  <tr>
    <td align="center"><img src="mocap_rig_overview.png" width="400"><br><em>Scene with the mocap rig: overhead and two oblique cameras on trusses</em></td>
    <td align="center"><img src="scene_view1.png" width="400"><br><em>Lite 6 arm and work plate on the table</em></td>
  </tr>
  <tr>
    <td align="center"><img src="mocap_view_cam_oblique_w.png" width="400"><br><em>View from the west oblique mocap camera</em></td>
    <td align="center"><img src="mocap_view_cam_overhead.png" width="400"><br><em>View from the overhead mocap camera</em></td>
  </tr>
  <tr>
    <td align="center"><img src="toolpath_result.png" width="400"><br><em>Commanded vs. actual tool-tip path on a static plate</em></td>
    <td align="center"><img src="mocap_demo_result.png" width="400"><br><em>Mocap estimate vs. true yaw, marker visibility and pose error</em></td>
  </tr>
  <tr>
    <td align="center" colspan="2"><img src="disturbance_check.png" width="600"><br><em>Scripted stepped-yaw disturbance: a smooth staircase with no X/Y drift</em></td>
  </tr>
</table>

## Quick start

Requires Python 3 with `mujoco`, `numpy`, `matplotlib` and `Pillow`:

```bash
pip install mujoco numpy matplotlib pillow

python check_reach.py                  # verify the arm can reach the whole plate
python trace_toolpath.py --view        # watch the toolpath on a static plate
python closed_loop_demo.py             # run open / closed / oracle and write the plots
python closed_loop_demo.py --view      # watch the closed loop live
```

In the live viewer, press `]` / `[` to cycle through the mocap camera views.

---

# Technical details

### Files

| File | Purpose |
|------|---------|
| `scene.xml` | The full scene (table + plate + arm include + lighting). Load this. |
| `ufactory_lite6/lite6.xml` | Lite 6 model, adapted for this scene (see changes below). |
| `check_reach.py` | Verifies the EE (`tcp` site) reaches a 9x9 grid over the plate via multi-seed IK. |
| `trace_toolpath.py` | Closed-loop *arm control* demo: IK+feedforward traces an ordered serpentine path across the (static) plate. Not to be confused with `closed_loop_demo.py`'s mocap-feedback loop. |
| `check_mocap_rig.py` | Verifies the emulated mocap camera rig's keep-out clearances and renders its views. |
| `mocap_rig_spec.md` | Reference spec for the emulated mocap rig (sensor, markers, camera poses, noise model). |
| `toolpath_lib.py` | Shared path-building + IK helpers used by `trace_toolpath.py` and `mocap_demo.py`. |
| `mocap_emulator.py` | Phase D noise/occlusion/dropout model for the mocap rig. |
| `mocap_demo.py` | Runs the toolpath while the plate wanders under phase E and the mocap emulator samples at 240 Hz; logs + plots the result. |
| `plate_disturbance.py` | Phase E: the scripted discrete stepped-yaw panel disturbance (PD torque on a real free body). |
| `check_disturbance.py` | Standalone verification of the disturbance alone (no arm). |
| `check_outlier_rejection.py` | Stress test proving the mocap fit's residual check catches a corrupted marker. |
| `closed_loop_demo.py` | Phase F: re-projects the toolpath through the live mocap estimate; compares open/closed/oracle tracking. |

### Scene layout (metres, world frame)

- **Table**: 1.0 (x) x 0.7 (y) box top, top surface at **z = 0.75**, four legs to the floor.
- **Work plate**: **0.30 x 0.30 m**, 12 mm thick, matte aluminum finish, **3 kg**
  (a real free body — `<freejoint/>`, resting on the table via ordinary
  contact, not a kinematic mocap body). Body at `(-0.03, 0, 0.756)` -> spans x
  in [-0.18, 0.12], y in [-0.15, 0.15], top surface at z = 0.762. Has 4 flush
  mocap markers at its corners (see "Emulated mocap rig" below).
  > Note: this is 0.30 m, not the originally-requested 2 ft (0.61 m). The Lite 6's
  > ~0.44 m usable reach at table height cannot cover a 0.61 m square from an
  > edge mount, so the plate was shrunk to stay fully inside the workspace.
- **Lite 6**: base mounted on the table at `(-0.30, 0, 0.75)`, facing +x toward the
  plate. Nearest plate edge is ~0.09 m clear of the base; far corners ~0.41 m away.

### Changes to `ufactory_lite6/lite6.xml` vs. the stock MuJoCo Menagerie model

- `link_base` given `pos="-0.30 0 0.75"` to mount it on the table.
- Asset dirs flattened (`assets/visual` -> `visual`, `assets/collision` -> `collision`)
  and `meshdir` dropped so meshes resolve when the file is `include`-d from `scene.xml`.
- Added a `tool` body under `link6`: a short probe shaft + red spherical tip with a
  `tcp` site at the contact point, for tool-path work.

### Verify reach

```
python check_reach.py
```

Expected: `RESULT: PLATE FULLY REACHABLE` (0 unreachable targets over the 9x9 grid).

### Tool-path tracing demo

```
python trace_toolpath.py           # headless: writes the GIF + result plot
python trace_toolpath.py --view    # live interactive window, loops the path (mouse to orbit/zoom)
```

To just look at the static scene:  `python -m mujoco.viewer --mjcf=scene.xml`

Traces an **ordered serpentine (boustrophedon) raster** over the plate:

1. **Path** — 7 lines along x, sweeping +/-y alternately, 25 mm inside the plate edge,
   1 mm above the surface, with a vertical lead-in / lead-out. Total on-surface
   length ~2.0 m.
2. **Trajectory** — the ordered knots are densified to a constant-feedrate
   (0.10 m/s) sequence of Cartesian set-points, one per 20 ms control tick.
3. **Closed-loop control** — each tick a 6-DOF damped-least-squares IK (own
   `MjData`, tool held pointing down) converts the pose error at a short
   look-ahead point into joint targets; the targets drive the Lite 6 **position
   actuators** and the physics is stepped. The look-ahead is feed-forward that
   cancels the position-loop lag.

Outputs:

| File | |
|------|---|
| `toolpath_trace.gif` | animation, red trail = executed path |
| `toolpath_result.png` | commanded vs. actual TCP (top view) + tracking error vs. time |

Typical result: **~2.5 mm mean** TCP tracking error on the straight passes,
peaking ~7 mm at the sharp 180 deg reversals (no corner deceleration).

### Emulated mocap rig

To capture the plate's position/orientation while it moves unpredictably during a
toolpath, the scene includes an emulated **OptiTrack PrimeX 22** marker-based mocap
setup: **4 flush 9.5 mm markers** at the plate's corners, and **3 static cameras**
external to the arm and workspace (1 overhead + 2 oblique on opposite vertical
trusses), all clear of the robot's swept volume and the "no hardware within 0.70 m
of table height" keep-out. See `mocap_rig_spec.md` for the sensor datasheet values,
marker layout, camera poses, and the keep-out/coverage math behind them.

```
python check_mocap_rig.py
```

Checks every camera's clearance from both keep-out zones and renders the 3 mocap
camera views plus one external overview (`mocap_view_cam_*.png`,
`mocap_rig_overview.png`).

To look through the rig's cameras live: `python trace_toolpath.py --view` (or
`python -m mujoco.viewer --mjcf=scene.xml` for the static scene), then press
**`]`** / **`[`** in the viewer window to cycle forward/backward through
`Free -> cam_overhead -> cam_oblique_w -> cam_oblique_e`. (`Tab` toggles the UI
side panels, not the camera — easy to mix up.)

The **noise/occlusion model (phase D)** is now implemented in `mocap_emulator.py`:
per-camera pixel-centroid noise, ray-traced occlusion dropout, calibration bias +
slow drift, marker label swaps, and ghost detections from reflections, feeding a
multi-view triangulation + rigid-body fit.

The **scripted panel disturbance (phase E)** is also implemented and wired into
the same run. The plate is now a **real MuJoCo free body** (`<freejoint/>`,
mass 3 kg — realistic for this 0.30 x 0.30 x 0.012 m aluminum plate), resting
on the table via ordinary contact, so gravity/contact/the arm's tool all
genuinely act on it. `plate_disturbance.py` drives it with a **discrete
stepped-yaw schedule**: 30 total steps, each ±1.5° (direction random per
step), separated by a random interval uniform in [2, 5] s, tracked by a PD
torque controller that settles smoothly between steps — no vibration, no
per-tick jitter, just a gentle staircase. (An earlier v1 used a continuous
Ornstein-Uhlenbeck random walk instead; replaced because it read as random
vibration rather than realistic motion — see `mocap_rig_spec.md`'s "Phase E"
section for that history, plus a real calibration gotcha this v2 hit: naive
PD gains sized from pure inertia were ~1000x too weak once the plate's contact
with the table was accounted for, not because of friction but because of how
MuJoCo's implicit integrator handles the redundant 4-corner box contact.)

```
python mocap_demo.py
```

Traces the toolpath while the plate steps through its schedule and the mocap
emulator samples at 240 Hz, and writes `mocap_demo_log.csv` (per-sample
true/estimated pose + error), `mocap_events.log` (every swap/ghost/dropout/
outlier-rejection/temporal-gate event), and `mocap_demo_result.png` (true vs.
estimated yaw, marker-visibility timeline, and pose error plots — the top
panel is a clean staircase, not noise). Typical result: 6 of the 30 scheduled
steps fire within the 23 s toolpath; the mocap estimate tracks the true yaw at
**~0.14 mm mean / 0.40 mm RMS** position error on valid frames (worst spike
~8 mm), with pose dropout at only **~0.9%** of frames. See `mocap_rig_spec.md`
for the full parameter table and validation numbers.

The rigid-body fit has two independent, additive robustness checks:
- **Outlier rejection** (spatial): after fitting, any marker whose residual
  against the fitted transform exceeds 2 mm gets dropped and the fit redone
  without it — only possible when a 4th, spare marker is available.
  `check_outlier_rejection.py` proves it with a synthetic corrupted marker.
- **Temporal gate**: each marker's new position is compared to *its own* last
  accepted position (never the other markers), so it still catches a bad
  marker even with only 3 available — the spatial check's blind spot. Its
  allowance grows with how long the marker's been missing, capped at 110 mm
  (a bug where that allowance grew unbounded for a long-missing marker was
  found via full-run replay and fixed). See `mocap_rig_spec.md`'s "Temporal
  gate" section for the calibration, the bug, and the accepted residual edge
  cases (rare, bounded-magnitude, and documented rather than chased further).

Together, in the current run, these caught 84 spatial + several temporal
corruption events (varies run to run with the RNG).

To check the disturbance alone (no arm): `python check_disturbance.py` —
verifies all 30 scheduled steps fire with the right magnitude/timing, settle
smoothly with no overshoot, and that the plate stays resting on the table with
zero X/Y drift throughout; plots the yaw staircase and translation drift
(`disturbance_check.png`).

### Closed-loop toolpath correction (phase F)

```
python closed_loop_demo.py                     # headless: writes CSV/plot for all 3 modes
python closed_loop_demo.py --view               # live viewer, closed-loop mode, loops the path
python closed_loop_demo.py --view --mode=open   # or --mode=oracle
```

`--view` opens the same live viewer as `trace_toolpath.py --view` (`]`/`[` to
cycle the mocap cameras still works), but with the plate actually stepping
through its yaw schedule — red trail = tool tip, blue trail = a plate corner
marker (traces a visible arc as it yaws; the plate's *center* no longer moves
at all under the v2 disturbance). `--mode` picks which of the three behaviors
below to watch; defaults to `closed`.

Re-projects the pre-planned path through the *live* mocap pose estimate every
control tick (`toolpath_lib.to_local_frame`/`apply_pose`), with the defensive
consumption phase D's own docs called for: hold the last correction through a
dropout, and a second, independent rate limiter (distinct from phase D's
per-marker temporal gate) protecting the controller from acting on a single bad
estimate. Runs three ways for a direct comparison against **true** plate
pose (never the estimate) — open-loop (today's pre-F behavior), closed-loop
(the real mocap-driven correction), and oracle (a hypothetical perfect,
zero-latency sensor) — and writes `closed_loop_log.csv` (world / robot-base /
plate-start frames, as yaw angle) and `closed_loop_result.png`.

| Mode | Mean contact error | Max |
|---|---|---|
| open | 5.39 mm | 14.5 mm |
| **closed** | **2.94 mm** | 19.2 mm (one brief transient, t ≈ 21.7 s) |
| oracle | 2.80 mm | 7.2 mm |

(From the committed `closed_loop_log.csv`, disturbance seed 2. An earlier run
recorded 6.46 / 2.71 / 2.77 mm mean.)

**Closing the loop cuts mean contact error by ~45%** and lands close to the
oracle (perfect-sensor) ceiling — a much clearer result than the ~11%
improvement seen under the old continuous-jitter disturbance (v1), where the
motion itself outran the arm's control bandwidth regardless of sensing
quality. With the gentler v2 stepped motion, the arm can actually keep up, and
correction recovers almost all of the achievable improvement; the ~2.8 mm
that's left over matches the arm's own baseline IK/actuator tracking lag on a
*static* plate — i.e. it's the arm's inherent precision limit, not the plate
motion or the mocap chain. See `mocap_rig_spec.md`'s "Phase F" section for
both results (v1 and v2) and the full analysis.

