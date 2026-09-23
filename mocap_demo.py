"""Phase D+E demo: run the toolpath while the plate wanders under the phase E
scripted disturbance, sampling the emulated mocap rig at 240 Hz.

Logs true vs. emulated plate pose, records occlusion/swap/ghost/dropout events,
and plots the result. This now exercises both phases together: the plate
genuinely moves (phase E), the arm follows the same open-loop programmed path
regardless (no reaction to the disturbance -- that's phase F), and phase D's
mocap emulator has to track a moving, not just static, rigid body.

Run: python mocap_demo.py
"""
import csv
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from toolpath_lib import build_serpentine_knots, densify, ArmIK, settle_to_start
from mocap_emulator import MocapEmulator, MocapScheduler, MocapConfig, MARKER_SITES
from plate_disturbance import PlateDisturbance, STEP_DEG, N_STEPS, INTERVAL_MIN_S, INTERVAL_MAX_S


def _yaw_deg(quat):
    """Standard yaw-Euler-angle extraction from a quaternion (exact for a pure
    Z-axis rotation, a very close approximation otherwise -- e.g. a mocap
    estimate with small off-axis noise)."""
    w, x, y, z = quat
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

# ----------------------------------------------------------------------------- setup
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
plate_id = model.body("plate").id

ik = ArmIK(model)
qadr, dadr = ik.qadr, ik.dadr

MARGIN, N_PASSES, SKIM, APPROACH = 0.025, 7, 0.001, 0.06
knots, corners, plate_top, pc, _ = build_serpentine_knots(
    model, margin=MARGIN, n_passes=N_PASSES, skim=SKIM, approach=APPROACH)

FEED, PLUNGE, SUBSTEPS = 0.10, 0.04, 10
CTRL_DT = model.opt.timestep * SUBSTEPS
traj = densify(knots, CTRL_DT, feed=FEED, plunge=PLUNGE)
LOOKAHEAD = 4
print(f"trajectory points = {len(traj)},  sim time ~ {len(traj) * CTRL_DT:.1f} s")

q_des = settle_to_start(model, data, ik, traj[0], qadr, dadr)

mocap = MocapEmulator(model, MocapConfig())
sched = MocapScheduler(mocap)
dist = PlateDisturbance(model, seed=2)
print(f"plate nominal pose: pos={np.round(dist.nominal_pos, 4)}  "
      f"(disturbance: {N_STEPS} steps of +/-{STEP_DEG} deg, random direction, "
      f"interval uniform [{INTERVAL_MIN_S}, {INTERVAL_MAX_S}] s; schedule spans "
      f"{dist.step_times[-1]:.1f} s total)")

# ------------------------------------------------------------------------- run + log
log_t, log_nvalid, log_posevalid = [], [], []
log_pos_err, log_rot_err = [], []
log_true_yaw, log_est_yaw = [], []   # true/estimated yaw from nominal, deg
events_all = []

t_sim = 0.0
for i, target in enumerate(traj):
    ff = traj[min(i + LOOKAHEAD, len(traj) - 1)]
    q_des = ik.solve(ff, q_des, iters=12)
    data.ctrl[:] = q_des
    for _ in range(SUBSTEPS):
        dist.apply(data, model.opt.timestep)     # advance + write this tick's torque
        mujoco.mj_step(model, data)
        t_sim += model.opt.timestep
        s = sched.maybe_sample(data, t_sim)
        while s is not None:
            events_all.extend(s.events)
            log_t.append(s.t_capture)
            log_nvalid.append(s.n_markers_valid)
            log_posevalid.append(s.pose_valid)
            true_pos = data.xpos[plate_id].copy()
            true_quat = data.xquat[plate_id].copy()
            log_true_yaw.append(_yaw_deg(true_quat))
            if s.pose_valid:
                log_pos_err.append(np.linalg.norm(s.pose_pos - true_pos) * 1000)
                log_est_yaw.append(_yaw_deg(s.pose_quat))
                qd = np.zeros(4)
                mujoco.mju_mulQuat(qd, s.pose_quat, np.array(
                    [true_quat[0], -true_quat[1], -true_quat[2], -true_quat[3]]))
                if qd[0] < 0:
                    qd = -qd
                log_rot_err.append(np.degrees(2 * np.arccos(np.clip(qd[0], -1, 1))))
            else:
                log_pos_err.append(np.nan)
                log_rot_err.append(np.nan)
                log_est_yaw.append(np.nan)
            s = sched.maybe_sample(data, t_sim)
    if i % 400 == 0:
        print(f"  tick {i}/{len(traj)}", flush=True)

log_t = np.array(log_t)
log_nvalid = np.array(log_nvalid)
log_posevalid = np.array(log_posevalid)
log_pos_err = np.array(log_pos_err)
log_rot_err = np.array(log_rot_err)
log_true_yaw = np.array(log_true_yaw)
log_est_yaw = np.array(log_est_yaw)

# ------------------------------------------------------------------------- summary
n_samples = len(log_t)
n_dropout = int((~log_posevalid).sum())
swap_events = [e for e in events_all if "label swap" in e]
ghost_events = [e for e in events_all if "ghost detection" in e]
marker_dropouts = [e for e in events_all if ": dropout" in e]
outlier_events = [e for e in events_all if "rejected as fit outlier" in e]

valid_pos = log_pos_err[log_posevalid]
valid_rot = log_rot_err[log_posevalid]
print("-" * 72)
print(f"true plate yaw from nominal: final {log_true_yaw[-1]:.3f} deg "
      f"(expected cumulative: {np.degrees(dist.targets[-1]):.3f} deg over "
      f"{dist._next_step}/{N_STEPS} steps fired so far), "
      f"range [{log_true_yaw.min():.3f}, {log_true_yaw.max():.3f}] deg")
print(f"mocap samples: {n_samples}  ({mocap.cfg.rate_hz:.0f} Hz nominal)")
print(f"pose dropouts (<{mocap.cfg.min_markers_per_pose} markers): "
      f"{n_dropout} ({100*n_dropout/n_samples:.2f}%)")
print(f"per-marker dropout events: {len(marker_dropouts)}")
print(f"label-swap events: {len(swap_events)}")
print(f"ghost-detection events: {len(ghost_events)}")
print(f"outlier-rejection events (bad marker caught + refit): {len(outlier_events)}")
if len(valid_pos):
    print(f"pose position error (valid frames): mean {valid_pos.mean():.3f} mm, "
          f"RMS {np.sqrt(np.mean(valid_pos**2)):.3f} mm, max {valid_pos.max():.3f} mm")
    print(f"pose rotation error (valid frames): mean {valid_rot.mean():.4f} deg, "
          f"max {valid_rot.max():.4f} deg")
print("-" * 72)

# ------------------------------------------------------------------------- outputs
with open("mocap_demo_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_capture_s", "n_markers_valid", "pose_valid", "pos_err_mm", "rot_err_deg",
                "true_yaw_deg", "est_yaw_deg"])
    for row in zip(log_t, log_nvalid, log_posevalid, log_pos_err, log_rot_err,
                   log_true_yaw, log_est_yaw):
        w.writerow(row)
print("wrote mocap_demo_log.csv")

with open("mocap_events.log", "w") as f:
    f.write("\n".join(events_all))
print(f"wrote mocap_events.log ({len(events_all)} events)")

fig, ax = plt.subplots(4, 1, figsize=(10, 11.5), sharex=True)

ax[0].plot(log_t, log_true_yaw, color="tab:green", lw=0.9, label="true (phase E)")
ax[0].plot(log_t, log_est_yaw, color="tab:orange", lw=0.7, ls="--", label="mocap estimate")
ax[0].set_ylabel("yaw from\nnominal [deg]")
ax[0].set_title("Phase E stepped-yaw disturbance: true vs. phase D mocap estimate")
ax[0].legend(fontsize=8, loc="upper right")

ax[1].plot(log_t, log_nvalid, color="tab:blue", lw=0.8)
ax[1].axhline(mocap.cfg.min_markers_per_pose - 0.5, color="tab:red", ls="--", lw=1,
              label=f"min markers for pose ({mocap.cfg.min_markers_per_pose})")
ax[1].set_ylabel("markers reconstructed"); ax[1].set_ylim(-0.3, 4.3)
ax[1].set_title("Marker visibility over the toolpath (occlusion dropout)")
ax[1].legend(fontsize=8, loc="lower right")

ax[2].plot(log_t, log_pos_err, color="tab:red", lw=0.8)
ax[2].set_ylabel("pose position error [mm]")
ax[2].set_title("Emulated mocap pose error vs. true plate pose (gaps = dropped frames)")

ax[3].plot(log_t, log_rot_err, color="tab:purple", lw=0.8)
ax[3].set_ylabel("pose rotation error [deg]"); ax[3].set_xlabel("time [s]")

fig.tight_layout()
fig.savefig("mocap_demo_result.png", dpi=110)
print("wrote mocap_demo_result.png")
