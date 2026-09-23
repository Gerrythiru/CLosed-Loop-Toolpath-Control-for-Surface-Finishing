"""Stress test for the Kabsch fit's outlier rejection (mocap_emulator._fit_rigid_robust).

We showed empirically (see the conversation / mocap_rig_spec.md) that a real
label swap essentially never fires on this rig -- the 4 corner markers are too
far apart relative to the camera distances for any two to land within the
swap-pixel-threshold, even at 89 deg of plate rotation. So instead of waiting
for one to occur naturally, this manufactures the *effect* a swap or a bad
ghost detection would have -- one marker's reconstructed position corrupted by
a large offset -- and checks that the residual check catches it.

Run: python check_outlier_rejection.py
"""
import numpy as np
import mujoco

from mocap_emulator import MARKER_SITES, MocapConfig, _fit_rigid_robust, _kabsch

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.mj_forward(model, data)

plate_id = model.body("plate").id
true_pos = data.xpos[plate_id].copy()
true_quat = data.xquat[plate_id].copy()

marker_site_ids = [model.site(n).id for n in MARKER_SITES]
local_offsets = np.array([model.site_pos[sid] for sid in marker_site_ids])
true_world = np.array([data.site_xpos[sid].copy() for sid in marker_site_ids])

cfg = MocapConfig()


def rot_err_deg(quat_est, quat_true):
    qd = np.zeros(4)
    mujoco.mju_mulQuat(qd, quat_est, np.array(
        [quat_true[0], -quat_true[1], -quat_true[2], -quat_true[3]]))
    if qd[0] < 0:
        qd = -qd
    return np.degrees(2 * np.arccos(np.clip(qd[0], -1, 1)))


print(f"corruption threshold (cfg.outlier_residual_m) = {cfg.outlier_residual_m*1000:.1f} mm")
print("-" * 78)

for corrupt_mm in [0, 1, 5, 20, 50, 100]:
    corrupted = true_world.copy()
    # corrupt one marker the way a swap would: replace it with (roughly) where a
    # different marker is, i.e. a large, plausible-looking wrong position.
    direction = (true_world[1] - true_world[0])
    direction /= np.linalg.norm(direction)
    corrupted[0] = true_world[0] + direction * (corrupt_mm / 1000.0)

    # ---- naive fit: all 4 markers, no rejection --------------------------------
    R_naive, t_naive = _kabsch(local_offsets, corrupted)
    q_naive = np.zeros(4)
    mujoco.mju_mat2Quat(q_naive, R_naive.ravel())
    naive_pos_err = np.linalg.norm(t_naive - true_pos) * 1000
    naive_rot_err = rot_err_deg(q_naive, true_quat)

    # ---- robust fit: with outlier rejection ------------------------------------
    R_rob, t_rob, used, rejected, resid = _fit_rigid_robust(
        local_offsets, corrupted, MARKER_SITES, cfg.min_markers_per_pose,
        cfg.outlier_residual_m)
    q_rob = np.zeros(4)
    mujoco.mju_mat2Quat(q_rob, R_rob.ravel())
    rob_pos_err = np.linalg.norm(t_rob - true_pos) * 1000
    rob_rot_err = rot_err_deg(q_rob, true_quat)

    caught = "mk_front_left" in [r[0] for r in rejected]
    print(f"corruption={corrupt_mm:4d} mm  |  naive: pos_err={naive_pos_err:7.3f} mm, "
          f"rot_err={naive_rot_err:6.3f} deg  |  robust: pos_err={rob_pos_err:7.3f} mm, "
          f"rot_err={rob_rot_err:6.3f} deg, rejected={rejected}, caught={caught}")

print("-" * 78)
print("Expected: naive error grows with corruption; robust fit rejects the bad")
print("marker once its residual clears the threshold and recovers the ~baseline")
print("(0 mm corruption) accuracy from the remaining 3 good markers.")
