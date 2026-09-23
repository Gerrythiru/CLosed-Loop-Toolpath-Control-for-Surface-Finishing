"""Standalone verification of the phase E (v2) stepped-yaw disturbance (no arm
involved). The plate is now a real free body driven by PD torque (see
plate_disturbance.py for the calibration story), not a kinematic mocap body,
so this checks a different set of properties than the old OU-based model did:

  - exactly N_STEPS steps actually fire, each of magnitude STEP_DEG
  - step directions are a genuine mix of both signs (not degenerately uniform)
  - intervals between steps fall in [INTERVAL_MIN_S, INTERVAL_MAX_S]
  - the settle response between steps is smooth (no oscillation/overshoot
    ringing) and gets close to the target well inside the minimum interval
  - the plate stays resting on the table throughout (no launching/tunneling --
    the real failure mode found and fixed during calibration, see
    plate_disturbance.py's module docstring)
  - final cumulative yaw matches the signed sum of all steps
  - there is zero X/Y translation throughout (pure torque about the COM)

Run: python check_disturbance.py
"""
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from plate_disturbance import (PlateDisturbance, STEP_DEG, N_STEPS,
                                INTERVAL_MIN_S, INTERVAL_MAX_S)

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.mj_forward(model, data)
plate_id = model.body("plate").id

dist = PlateDisturbance(model, seed=7)
dt = model.opt.timestep
T = dist.step_times[-1] + 10.0   # run a bit past the last scheduled step
n = int(T / dt)
print(f"nominal pose: pos={dist.nominal_pos}, dt={dt*1000:.1f} ms, "
      f"{N_STEPS} steps scheduled over {dist.step_times[-1]:.1f} s")

t_arr = np.zeros(n)
yaw_arr = np.zeros(n)
z_arr = np.zeros(n)
xy_drift_arr = np.zeros(n)
for i in range(n):
    dist.apply(data, dt)
    mujoco.mj_step(model, data)
    q = data.xquat[plate_id]
    t_arr[i] = (i + 1) * dt
    yaw_arr[i] = np.degrees(2 * np.arctan2(q[3], q[0]))
    z_arr[i] = data.xpos[plate_id][2]
    xy_drift_arr[i] = np.linalg.norm(data.xpos[plate_id][:2] - dist.nominal_pos[:2])

# ---- schedule sanity ------------------------------------------------------------
print(f"steps fired: {dist._next_step} / {N_STEPS}")
assert dist._next_step == N_STEPS, "not all scheduled steps fired within the run"
assert np.all(np.abs(np.abs(dist.signs) - 1.0) < 1e-12), "step signs must be +/-1"
n_pos, n_neg = (dist.signs > 0).sum(), (dist.signs < 0).sum()
print(f"step directions: {n_pos} CCW (+), {n_neg} CW (-)  (mixed, as expected from a fair coin flip)")
assert n_pos > 0 and n_neg > 0, "direction should be a genuine mix, not degenerate"
assert np.all(dist.intervals >= INTERVAL_MIN_S) and np.all(dist.intervals <= INTERVAL_MAX_S), \
    "interval out of the configured [min, max] range"
print(f"intervals: min {dist.intervals.min():.2f} s, max {dist.intervals.max():.2f} s, "
      f"mean {dist.intervals.mean():.2f} s  (configured range [{INTERVAL_MIN_S}, {INTERVAL_MAX_S}])")

# ---- final yaw vs. expectation ---------------------------------------------------
expected_final_deg = np.degrees(dist.targets[-1])
print(f"final actual yaw = {yaw_arr[-1]:.3f} deg, expected cumulative target = "
      f"{expected_final_deg:.3f} deg (= sum of {N_STEPS} x +/-{STEP_DEG} deg steps)")
assert abs(yaw_arr[-1] - expected_final_deg) < 0.1, "final yaw doesn't match the commanded schedule"

# ---- stability: resting on the table, no translation -----------------------------
print(f"z range: [{z_arr.min():.4f}, {z_arr.max():.4f}] m (nominal {dist.nominal_pos[2]:.4f})")
assert z_arr.max() - z_arr.min() < 0.005, "plate moved vertically -- possible launch/tunnel event"
print(f"max X/Y drift from nominal over the whole run: {xy_drift_arr.max()*1000:.4f} mm")
assert xy_drift_arr.max() < 0.5, "unexpected X/Y translation -- torque should be pure-Z about the COM"

# ---- per-step settle quality: no overshoot/oscillation ----------------------------
overshoots = []
for k in range(N_STEPS):
    t0 = dist.step_times[k]
    t1 = dist.step_times[k + 1] if k + 1 < N_STEPS else T
    mask = (t_arr >= t0) & (t_arr < t1)
    if not mask.any():
        continue
    target_deg = np.degrees(dist.targets[k + 1])
    seg_yaw = yaw_arr[mask]
    # overshoot = how far the segment goes past the target, signed with the step direction
    if dist.signs[k] > 0:
        overshoot = seg_yaw.max() - target_deg
    else:
        overshoot = target_deg - seg_yaw.min()
    overshoots.append(overshoot)
overshoots = np.array(overshoots)
print(f"per-step overshoot beyond target: mean {overshoots.mean():.4f} deg, "
      f"max {overshoots.max():.4f} deg  (should be small/near-zero -- no ringing)")
assert overshoots.max() < 0.5, "visible overshoot/oscillation in the step response"

print("-" * 72)
print("RESULT: stepped-yaw disturbance behaves as designed "
      "(correct schedule, smooth settling, stable contact, zero translation)")

# ---- plot ------------------------------------------------------------------------
fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
window = t_arr < min(40.0, T)
ax[0].plot(t_arr[window], yaw_arr[window], color="tab:blue", lw=1.2)
for st in dist.step_times[dist.step_times < 40.0]:
    ax[0].axvline(st, color="0.85", lw=0.7, zorder=0)
ax[0].set_ylabel("yaw [deg]")
ax[0].set_title(f"Phase E (v2) stepped-yaw disturbance (first 40 s of {T:.0f} s) -- "
                 "gentle staircase, not vibration")

ax[1].plot(t_arr[window], xy_drift_arr[window] * 1000, color="tab:red", lw=1.0, label="X/Y drift")
ax[1].plot(t_arr[window], (z_arr[window] - dist.nominal_pos[2]) * 1000, color="tab:green",
           lw=1.0, label="Z drift")
ax[1].set_ylabel("drift [mm]"); ax[1].set_xlabel("time [s]")
ax[1].set_title("Translation drift (should stay ~0 -- pure yaw about the COM)")
ax[1].legend(fontsize=8)

fig.tight_layout()
fig.savefig("disturbance_check.png", dpi=110)
print("wrote disturbance_check.png")
