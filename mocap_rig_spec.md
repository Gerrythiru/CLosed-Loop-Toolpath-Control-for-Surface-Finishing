# Emulated mocap rig — reference spec

Physical layout for the emulated OptiTrack marker-based mocap system added to
`scene.xml`. This is the constants reference for later phases (noise/occlusion
model, panel disturbance, logging) — see `README.md` for how it fits into the
overall scene.

## Sensor: OptiTrack PrimeX 22

Source: [optitrack.com/cameras/primex-22](https://www.optitrack.com/cameras/primex-22),
[specs](https://www.optitrack.com/cameras/primex-22/specs.html).

| Spec | Value |
|---|---|
| Sensor | 2.2 MP, 2048 x 1088 px (aspect ~1.88:1) |
| Lens (this build) | 6.5 mm, F1.6, wide option -> **79° horizontal / 47° vertical FOV** |
| Native frame rate | 360 FPS |
| Latency | 2.8 ms (camera-side) |
| Accuracy | < ±0.15 mm position, < 0.5° rotation (typical calibrated volume) |
| Passive marker range | ~70 ft (21 m) |

`scene.xml` sets `fovy="47"` on each `<camera>` to match the vertical FOV; when
rendering, use an aspect ratio close to 1.88:1 (e.g. 544 x 1024) to match the real
sensor's framing.

The *operating* point used later for the noise/latency/dropout emulation (240 Hz
output, ~0.15 mm 3D RMS, ~8 ms end-to-end latency including streaming) reuses these
datasheet numbers rather than re-deriving them — that work is phase D, not yet
implemented.

## Panel markers

- 4 markers, one per plate corner, **9.5 mm diameter, flush-mounted** (modeled as a
  thin ~1 mm disc rather than a marker-on-post, since a protruding marker would be
  clipped by the tool skimming the surface).
- Inset 10 mm from each physical edge (plate half-extent 0.15 m -> local centers at
  `(±0.14, ±0.14)`, z = +0.006, the plate's top surface).
- Named relative to the arm: local **+x = front** (far/distal edge, away from the
  arm base), **-x = back** (near/proximal edge); **+y/-y = left/right**.

| Site | Local pos (plate frame) |
|---|---|
| `mk_front_left`  | `( 0.14,  0.14, 0.006)` |
| `mk_front_right` | `( 0.14, -0.14, 0.006)` |
| `mk_back_left`   | `(-0.14,  0.14, 0.006)` |
| `mk_back_right`  | `(-0.14, -0.14, 0.006)` |

Each has a coincident `<site>` (for per-marker ground-truth sensors in a later
phase) and a visual `_geom` disc using the `marker` material.

## Camera rig: 1 overhead + 2 oblique

**Design constraints:**
- Keep-out A (table height): no camera mounted at or below `table_top + 0.70 m`
  = **z ≤ 1.45 m** (table top z = 0.75 m).
- Keep-out B (arm sweep): outside a sphere at the Lite 6 shoulder,
  center `(-0.30, 0, 0.9935)`, radius **0.60 m** (kinematic max reach ~0.55 m +
  tool + margin).
- Coverage target: table + plate + arm-reach envelope, bounding box
  `x:[-0.65, 0.60]`, `y:[-0.45, 0.45]`, `z:[0.75, 1.60]` m.

| Camera | Mount | Position (m) | Aim point (m) | Height above table | Arm-sphere clearance |
|---|---|---|---|---|---|
| `cam_overhead` | overhead truss beam, z=2.08 | `(0.00, 0.00, 2.00)` | `(-0.05, 0, 0.85)` | 1.25 m | 0.45 m |
| `cam_oblique_w` | vertical pole, west corner | `(-1.00, -0.90, 1.60)` | `(-0.10, 0, 0.85)` | 0.85 m | 0.69 m |
| `cam_oblique_e` | vertical pole, east corner | `(0.65, 0.95, 1.60)` | `(-0.05, 0, 0.85)` | 0.85 m | 0.87 m |

All three clear both keep-outs (verified numerically in `check_mocap_rig.py`). The
oblique cameras sit at diagonally opposite corners (~2.7 m apart) for a wide
triangulation baseline, so the arm swinging through one oblique view still leaves
the other oblique view plus the overhead view on the plate — needed since a rigid-
body solve requires ≥3 of the 4 corner markers visible to ≥2 of the 3 cameras
(occlusion accounting itself is a later phase; this geometry is chosen to make it
achievable).

`xyaxes` for each camera was computed with a standard look-at (world-up `(0,0,1)`,
except `(0,1,0)` for the near-vertical overhead camera to avoid the singularity).

**Known cosmetic item:** the overhead beam sits close to the directional overhead
light and casts a thin (~6 cm) shadow across part of the plate in renders from
`cam_overhead`. It doesn't obscure any marker. If it matters later (e.g. for a
marker-detection-realism pass), fix by offsetting the beam off the plate's
centerline or adding a fill light — not done here since it doesn't affect the rig
geometry itself. Camera mounting brackets (pole/beam -> housing) are not modeled;
there's a small unconnected visual gap in the overview render.

## Files

| File | Purpose |
|---|---|
| `scene.xml` | Markers (`mk_*`, `mk_*_geom`) on `plate`; truss geoms + 3 `<camera>`s in body `mocap_rig`. |
| `check_mocap_rig.py` | Verifies keep-out clearances; renders the 3 camera views + one overview. |
| `mocap_view_cam_*.png`, `mocap_rig_overview.png` | Verification renders (regenerate with `check_mocap_rig.py`). |
| `toolpath_lib.py` | Shared path-building + IK, factored out of `trace_toolpath.py` for reuse by `mocap_demo.py`. |
| `mocap_emulator.py` | Phase D: the noise/occlusion/dropout model (`MocapEmulator`, `MocapConfig`, `MocapScheduler`). |
| `calibrate_mocap_noise.py` | One-off: picks the per-camera pixel-noise sigma to hit the ~0.15 mm 3D RMS target. |
| `check_outlier_rejection.py` | Stress test proving the Kabsch fit's residual check catches a corrupted marker. |
| `mocap_demo.py` | Runs the toolpath while sampling the emulator at 240 Hz; writes the CSV/plot/event log below. |

## Phase D: noise/occlusion/dropout model

Implemented in `mocap_emulator.py`. Pipeline per mocap sample: project each true
marker into each camera's pixel plane -> occlusion ray-test (`mj_ray`, arm/plate/
table) -> marker-swap check for markers close together in one camera's image ->
ghost-detection injection for occluded markers -> per-camera pixel noise -> per-
marker multi-view DLT triangulation (needs >=2 contributing cameras) -> global
calibration bias + slow drift -> Kabsch rigid-body fit (needs >=3 of 4 markers).

| Parameter | Value | Source |
|---|---|---|
| Output rate | 240 Hz | agreed operating point (< PrimeX 22's 360 Hz native) |
| End-to-end latency | 8 ms | agreed operating point (> the camera's own 2.8 ms) |
| Per-camera pixel noise | 0.1014 px (1 sigma) | `calibrate_mocap_noise.py`, tuned so 3-camera triangulation gives ~0.15 mm 3D RMS |
| Calibration bias | 0.10 mm fixed offset, random direction per run | matches "constant calibration bias" |
| Slow drift | 0.05 mm amplitude, 90 s period | matches "optional slow drift" |
| Marker-swap trigger | <3 px separation in one camera's image, 30% swap probability | |
| Ghost-detection rate | 1%/camera/frame per occluded marker, +/-15 px jitter | |
| Rigid-body requirement | >=3 of 4 markers, each seen by >=2 of 3 cameras | matches the rig design requirement above |
| Outlier-rejection threshold | 2 mm Kabsch fit residual | see "Outlier rejection" below |
| Temporal-gate limit | 3mm + 3.45 m/s x gap, capped at 110 mm | see "Temporal gate" below |

**Gotcha found and fixed:** `mj_ray` ignores `contype`/`conaffinity` and filters
only by geom group, so the camera-housing decoration geoms (sitting a few cm from
their own camera) were self-occluding every ray. Fix: housings moved to their own
geom group (4), excluded from the occlusion raycast's group mask.

**Validation run** (`mocap_demo.py`, static plate — see caveat below): 5521
samples over the 23 s toolpath.

| Metric | Result |
|---|---|
| Pose dropouts (<3 markers) | 100 / 5521 (1.8%) |
| Per-marker occlusion dropouts | 3499 |
| Ghost-detection events | 155 |
| Label-swap events | 0 (see caveat) |
| Pose position error, valid frames | mean 0.24 mm, RMS 0.94 mm, max 21.6 mm |
| Pose rotation error, valid frames | mean 0.09 deg, max 7.1 deg |

The error floor sits near the calibrated ~0.15 mm target; the spikes (up to ~22 mm)
occur when the fit falls back to exactly 3 markers and one contributing detection
is a ghost or a marginal view — i.e. the model reproduces the real failure mode
(fewer, worse-conditioned points -> larger, occasionally large-outlier error)
rather than just adding flat Gaussian noise. Outputs: `mocap_demo_log.csv`
(per-sample), `mocap_events.log` (every swap/ghost/dropout), `mocap_demo_result.png`
(marker-visibility timeline + pose error plots).

**Caveat — zero swap events:** the plate is still static (phase E isn't
implemented), so each camera's marker-to-marker pixel separations never change
during the run; if they're never below the 3 px swap threshold at the start, they
never are. Swaps should start appearing once phase E adds plate rotation that can
turn the plate edge-on to a camera.

## Phase E (v2): discrete stepped-yaw disturbance on a real physics body

Implemented in `plate_disturbance.py` (`PlateDisturbance`). Superseded the v1
continuous Ornstein-Uhlenbeck random walk on a kinematic mocap body — per the
user, that read as random vibration, not realistic motion, and a mocap body
has no mass/inertia so nothing was ever really *simulated*. v2 is a sequence
of **discrete yaw steps** on a **real free body**.

**The plate is a genuine dynamic body now**: `plate` in `scene.xml` has a
`<freejoint/>` and `plate_geom` carries a real mass (see below), resting on the
table via ordinary contact — gravity, normal force, friction all apply, and
anything (the arm's tool, in particular) can physically push it. `apply()`
writes a torque into `data.xfrc_applied` every physics step; MuJoCo's own
rigid-body dynamics does the rest.

| Parameter | Value |
|---|---|
| Motion | Discrete yaw steps, pivoting about the plate's own origin (== COM) |
| Step size | ±1.5°, direction random per step |
| Step count | 30 total (fixed schedule, pre-generated from a seeded RNG) |
| Interval between steps | uniform random in [2, 5] s |
| Mass | 3 kg (realistic for a 0.30 x 0.30 x 0.012 m aluminum plate — matches ~2.9 kg calculated from density) |
| Izz (about the yaw axis) | 0.045 kg·m² (= m·(a²+b²)/12, matches MuJoCo's auto-derived inertia from the geom) |

**Mass history**: this pass first tried 80 kg, then 30 kg (both explicitly
requested), before settling on the physically-realistic **3 kg**. The PD gains
below are calibrated specifically for 3 kg + the softened contact — they do
not carry over if the mass changes again; re-run the calibration sweep
described in `plate_disturbance.py`'s module docstring.

**A real, non-obvious calibration problem, not just "pick some gains"**: a
naive PD controller sized from pure rotational-inertia dynamics (Izz·ω²,
critically damped) under-drove the plate by ~3 orders of magnitude. The root
cause wasn't friction — verified by reproducing the identical suppression with
`condim="1"` (a purely frictionless normal-only contact). It's that a flat box
resting on a flat table via **4 simultaneous corner contacts** is a classic
redundant/over-constrained configuration for a rigid-body solver, and combined
with the `implicitfast` integrator (kept globally for the arm's own stability)
this absorbed nearly all applied torque as spurious numerical damping below a
threshold, and made the contact solver fail outright (visible launching /
tunneling of the plate) just above it — no usable middle ground with the
default contact stiffness. Fix: soften `plate_geom`'s `solref`/`solimp` in
`scene.xml`. That alone turned the response clean, linear, and stable across a
wide torque range, at which point a conventional PD design worked:

| Parameter | Value |
|---|---|
| `solref` / `solimp` (plate-table contact) | `0.05 1` / `0.9 0.95 0.01 0.5 2` (softened from MuJoCo defaults) |
| `KP_ROT` | 134 N·m/rad |
| `KD_ROT` | 6 N·m/(rad/s) |
| `TORQUE_CAP` | 3.5 N·m (safety margin below the ~5 N·m instability threshold found by sweeping constant torque) |

Pivot is the plate's own origin (== COM) for this pass, so pure Z-axis torque
at the COM is exact — it changes only angular momentum, not linear momentum,
so the plate spins in place with zero translation forced by the controller
(confirmed: 0.000 mm max X/Y drift over a full 30-step, 103 s run). A "V2"
pivoting about a plate corner instead — named by the user as a explicit
follow-up, not built here — would need a second, coupled PD *force* loop
tracking the COM around the arc an off-center pivot implies, i.e.
reconstructing what a real hinge joint gives for free.

**Standalone verification** (`check_disturbance.py`, full 30-step/103 s
schedule, no arm):

| Check | Result |
|---|---|
| Steps fired | 30 / 30 |
| Step directions | genuine mix (15 CW / 15 CCW for the doc's seed) |
| Intervals | all within [2.0, 5.0] s |
| Final yaw vs. commanded schedule | matches to <0.1° |
| Per-step overshoot | ~0 (max 0.0002°) — no ringing, settles from *below* the target, never past it |
| Z range over the whole run | 0.4 mm (settling only, then flat) — stays resting, no launch/tunnel |
| Max X/Y drift over the whole run | 0.000 mm |

**Combined D+E validation** (`mocap_demo.py`, 23 s toolpath): 6 of the 30
scheduled steps fire within the window. Pose dropout **0.89%** of frames (down
from 1.0% under the v1 X/Y+yaw model, similar order — this motion is gentler
so markers rarely leave frame). The disturbance is now visually and
numerically a smooth staircase (see `mocap_demo_result.png`'s top panel) — no
per-tick jitter at all, matching the user's "no vibration" requirement
directly.

Label-swap events are still 0 — the 3 px in-frame separation threshold is
still never met between the four 280 mm-apart corner markers from these camera
distances/angles, whether the plate wanders continuously (v1) or steps
discretely (v2). Sweeping plate rotation up to 89° (far past anything this
30-step x 1.5° schedule reaches even in the all-same-direction extreme, 45°)
the closest any two markers ever get in any camera is ~26 px — still 9x the
swap threshold. This rig's marker spacing vs. camera distance makes a swap
essentially unreachable regardless of how the plate is driven; see "Outlier
rejection" and "Temporal gate" below for how we validated that failure mode
anyway.

## Outlier rejection (Kabsch fit residual check)

Since a real label swap can't be exercised on this rig (see above), the risk it
represents — a marker's reconstructed position silently corrupted by a large,
wrong-but-plausible offset, producing a `pose_valid=True` result that is
actually wrong — is real regardless of *how* the corruption happens (a swap on
a different rig, a bad ghost detection, anything). `_fit_rigid_robust()` in
`mocap_emulator.py` closes that gap: after the initial Kabsch fit, each
contributing marker's residual (distance between its reconstructed position and
where the fitted rigid transform says it should be) is checked against
`outlier_residual_m` (2 mm — well above the ~0.15 mm calibrated noise floor,
well below realistic corruption magnitudes). If there's a spare marker above
the 3-marker minimum and its residual exceeds threshold, it's dropped and the
fit is redone with the rest. With only 4 markers total this is at most one
rejection per frame (4 -> 3); at exactly 3 markers there's no spare point to
check against, so rejection isn't possible (matches the sensor design
requirement of needing 4 for full redundancy).

**Stress test** (`check_outlier_rejection.py`): one marker's true position is
displaced by a known amount and fit both with and without rejection.

| Corruption | Naive fit error | Robust fit error | Caught? |
|---|---|---|---|
| 0 mm (baseline) | 0.000 mm / 0.000° | 0.000 mm / 0.000° | n/a |
| 1 mm (below threshold) | 0.250 mm / 0.051° | 0.250 mm / 0.051° | no (correctly — within noise floor) |
| 5 mm | 1.250 mm / 0.257° | 0.000 mm / 0.000° | **yes** |
| 20 mm | 5.000 mm / 1.042° | 0.000 mm / 0.000° | **yes** |
| 100 mm | 25.000 mm / 5.599° | 0.000 mm / 0.000° | **yes** |

The robust fit recovers exact baseline accuracy the moment the corruption clears
the threshold, confirming the mechanism works before ever seeing a real swap.

**Effect on the real ±50 mm D+E run** (measured before the bound was revised to
±30 mm; see "Temporal gate" below for the current combined numbers):
re-running `mocap_demo.py` with rejection enabled caught **86 real corruption
events** — all from ghost detections occasionally corrupting a 4-marker fit —
and measurably improved accuracy:

| Metric | Without rejection | With rejection |
|---|---|---|
| Position error (valid frames), mean / RMS | 0.22 / 0.78 mm | **0.13 / 0.33 mm** |
| Position error, max | 20.3 mm | **10.4 mm** |
| Rotation error, max | 7.0° | **3.9°** |

So even though the *swap* mechanism specifically never fires on this rig, the
robustness gap it revealed was real — the ghost-detection failure mode was
already occasionally corrupting 4-marker fits, undetected, before this fix.

## Temporal gate (per-marker plausibility check)

> **Note (post phase-E-v2):** the calibration numbers in this section were
> measured against the old continuous OU disturbance. The v2 stepped-yaw
> motion is much gentler (smooth ramps over ~1-3 s rather than per-tick
> jitter), so this threshold is now almost certainly looser than strictly
> necessary — not unsafe, just not re-optimized for the new motion profile.
> Re-run the same empirical calibration method described below against v2 if
> tightening it becomes worthwhile.

The outlier-rejection check above is structurally blind whenever exactly 3
markers are available (no spare point to cross-check against) — and we found
`mocap_demo.py` runs where every one of the worst error spikes was exactly that
case: a marker already occlusion-dropped, leaving 3 candidates, one of which was
ghost-corrupted. `mocap_emulator.py` closes that gap with a **second,
independent** check that runs *before* the spatial one: each marker's freshly
triangulated position is compared only to **its own last accepted position**
(never to the other markers), so it works even down to a single marker with no
"spare" required.

```
limit = min(temporal_base_m + temporal_speed_mps * dt_gap, temporal_envelope_cap_m)
reject the marker this frame if its jump since last accepted > limit
```

**Calibration** (`temporal_base_m=3mm`, `temporal_speed_mps=3.45`, giving a
~17.4 mm limit at the nominal 240 Hz tick spacing): picked empirically, the same
way as everything else in this pipeline — we measured the *genuine* (noiseless)
per-tick marker motion under the actual disturbance and set the limit ~25% above
its observed max, so it doesn't fire on real motion:

| Bound | Genuine per-tick marker motion (p99.9 / max) |
|---|---|
| ±50 mm | 20.4 / 22.9 mm |
| ±30 mm (current) | 12.5 / 13.9 mm |

This is also *why* the bound was revised to ±30 mm: at ±50 mm, genuine motion
(max ~23 mm) and a typical ghost jump (~24 mm, see the worked example in the
conversation) were too close in magnitude for any fixed threshold to reliably
separate them without either missing corruption or false-flagging real motion.
At ±30 mm they're cleanly separated.

**A bug found by full-pipeline replay, not by the calibration alone**: the
`base + speed * dt_gap` formula grows without bound for a marker that's been
missing a long time. Replaying the real run, we found `mk_back_left` occluded
for **1.0 s** before a ghost fired for it — at that gap the formula's limit had
grown to ~3.45 m, i.e. no limit at all. Fixed with `temporal_envelope_cap_m`
(0.11 m — just above the ±30 mm bound's worst-case 3-axis round trip,
`2*sqrt(3)*30mm ≈ 103.9 mm`): once a gap exceeds a few correlation times
(~0.08 s here), the marker has fully decorrelated from its old position, so
"how long has it been gone" stops being useful information — the only honest
ceiling left is "still somewhere in the known envelope," not an ever-growing
allowance.

**Known, accepted residual limitation** (documented, not chased further): even
with the cap fixed, two related edge cases can still slip through, both because
*no* per-marker history is informative at that point:
- **Short-gap near-miss**: a marker dropped for only 1-2 ticks, corrupted, whose
  jump happens to land just under the ~17.4 mm threshold. Inherently bounded to
  roughly the threshold's own scale (a few mm over it at most) — 9 of the 10
  worst frames in the ±30 mm run are this case, none exceeding ~10 mm.
- **Long-gap double-occlusion**: two markers occluded simultaneously for an
  extended period (here, ~1 s) with a ghost firing for one right as it
  reappears. After that long a gap the marker has genuinely decorrelated, so
  temporal history carries no signal — and the spatial check is also blind
  here since only 2-3 real markers are up. This fired **once** in the 23 s
  ±30 mm run, producing the run's single worst spike (10.0 mm) — bounded by the
  ghost mechanism's own jitter scale, not literally unbounded, even before the
  cap fix.

**Combined D+E+D-robustness validation** (`mocap_demo.py`, temporal gate +
outlier rejection both active): 5521 samples over 23 s. The finding-the-bug
numbers above (10.7% dropout etc.) were from the older, full 6-DOF ±30 mm
disturbance; the table below is the current, corrected X/Y+yaw ±30 mm run:

| Metric | Result |
|---|---|
| Pose dropouts (<3 markers) | 56 / 5521 (1.0%) |
| Ghost-detection events | 157 |
| Outlier-rejection events (spatial check) | 101 |
| Temporal-gate rejection events | 24 |
| Position error (valid frames), mean / RMS / max | 0.13 / 0.35 / **10.7 mm** |
| Rotation error (valid frames), mean / max | 0.047° / 3.4° |

Pinning z/roll/pitch to 0 markedly reduced dropout (10.7% -> 1.0%) since
markers now leave a camera's frame or fall into occlusion far less often — the
disturbance is smaller in scope even though the x/y/yaw bounds themselves are
unchanged. The spatial/temporal check counts and worst-case error are the same
order of magnitude as before, consistent with the residual-limitation analysis
above still applying (same underlying mechanisms, just rarer now).

**Recommendation carried into phase F**: rather than continuing to chase these
last, rare, bounded-magnitude edge cases inside the sensor model, phase F's
consumption of the pose estimate should be defensive by design — hold the last
correction when `pose_valid=False`, and rate-limit / sanity-check any single-tick
jump in the *corrected* pose before acting on it. That's a second, independent
layer of protection against this whole family of rare spikes, and it's needed
anyway for the already-documented dropout/outlier cases regardless of how tight
D's own checks get.

## Phase F: closed-loop toolpath correction

Implemented in `closed_loop_demo.py`, using two new `toolpath_lib.py` helpers:
`to_local_frame`/`apply_pose` (exact inverses of each other, and of the rigid
transform `mocap_emulator._kabsch` already fits — re-projecting a local
waypoint through a live pose estimate is that fit run in reverse). The
pre-planned path is converted once to the plate's own local frame; every
control tick, `world_target = apply_pose(held_pos, held_quat, local_traj[i])`
re-projects it through a **held** pose estimate that only updates from a mocap
sample once `t_sim >= t_available` (respects latency) and `pose_valid` (holds
indefinitely through a dropout, per the design decision), and only within a
**second, independent rate limiter** (`CORR_RATE_*` in `closed_loop_demo.py`) —
distinct from phase D's per-marker temporal gate, which protects the sensor
*estimate*; this one protects the *controller* from acting on a single bad
estimate that slipped through anyway. Calibrated the same way as everything
else: genuine plate-origin motion (p99.9 ~11.6 mm, max ~13.0 mm per tick) with
~25% margin. *(Calibrated against the v1 OU disturbance, like the temporal
gate above — looser than strictly necessary for v2's gentler steps, but still
safe; 0 rate-limiter rejections in the v2 runs below.)* The look-ahead
feedforward reacts to the latest held estimate only (no plate-motion
prediction), per the design decision.

**The real proof point** isn't mocap accuracy (phase D already validated that)
but whether the tool actually ends up where the *true, moving* plate says it
should — measured every control tick against ground truth, never the estimate.
Run three ways for comparison: **open** (today's pre-F behavior, held pose
pinned to nominal), **closed** (the real mocap-driven correction above), and
**oracle** (held pose = ground truth every physics step, zero latency/noise —
the best any correction scheme could possibly achieve).

**Result under the v1 continuous OU disturbance** (historical — kept for the
finding, which is still relevant background even though the disturbance model
has since changed):

| Mode | Mean | RMS | Max |
|---|---|---|---|
| open | 12.55 mm | 14.14 mm | 37.4 mm |
| closed | 11.16 mm | 12.61 mm | 36.1 mm |
| oracle | 10.57 mm | 11.93 mm | 34.0 mm |

Closed-loop landed within 5.6% of the oracle ceiling (phase D's sensing was
already near-optimal), but the improvement over open-loop was modest (~11%,
closing 70% of the open->oracle gap) because the OU disturbance's per-tick
jitter (up to ~13 mm every 4 ms) was simply faster than the Lite 6's control
loop (IK solve + position actuators at 50 Hz) could track, *regardless* of
sensing quality — confirmed by testing with the look-ahead feedforward
disabled, which made tracking *worse* (13.72 mm mean), ruling that out as an
alternative explanation.

**Result under the v2 stepped-yaw disturbance** (current model — smooth,
gentle steps well within the arm's control bandwidth):

| Mode | Mean | RMS | Max |
|---|---|---|---|
| open | 6.46 mm | 8.07 mm | 17.8 mm |
| **closed** | **2.71 mm** | **2.92 mm** | 7.1 mm |
| oracle | 2.77 mm | 2.98 mm | 7.1 mm |

Night and day difference: closing the loop now cuts mean contact error by
**58%** (vs. ~11% under v1), and lands *at* the oracle ceiling (closed-loop was
actually fractionally better than oracle here, well within run-to-run RNG
noise). This confirms the earlier diagnosis directly — the v1 result wasn't a
sensing limitation, it was the disturbance outrunning the arm's control
bandwidth; give the arm a trackable (if still "unpredictable" from the arm's
own open-loop perspective) motion, and correction essentially recovers full
oracle-level performance. The residual ~2.7 mm even in the oracle case matches
the arm's own baseline IK/actuator tracking lag on a *static* plate (established
back in `trace_toolpath.py` at ~2.5 mm mean) — i.e. what's left over is the
arm's inherent tracking precision, not anything to do with the plate motion or
the mocap chain at all.

**Logging** (`closed_loop_log.csv`): per control tick, both true and held
(estimated) plate yaw in all three requested reference frames — world
(absolute), robot base (`link_base` body frame), and plate-start/nominal frame
— using `to_local_frame`/a relative-quaternion yaw extraction for the
transforms. (Position displacement isn't a useful signal to log any more since
v2: the plate only yaws about its own origin, so its center never translates.)

**Live viewer** (`closed_loop_demo.py --view [--mode=open|closed|oracle]`):
watch the disturbance and correction happen in real time (same viewer as
`trace_toolpath.py --view`), red trail on the tool tip, blue trail on a plate
corner marker (`mk_front_left`) — traces a visible arc as the plate yaws,
unlike the plate center, which no longer moves at all. Shares its per-tick
held-pose logic verbatim with `run()`'s headless path.

## Deferred

Nothing from the original phase D/E/F scope remains — this section is kept
empty intentionally rather than removed, as a marker for any future phase.
