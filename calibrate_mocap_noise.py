"""One-off calibration: pick the per-camera pixel-centroid noise sigma so that,
triangulated through the actual 3-camera rig geometry with all cameras unoccluded,
the reconstructed 3D marker position has ~0.15 mm RMS error -- the PrimeX 22's
quoted calibrated-volume accuracy (see mocap_rig_spec.md).

Triangulation error scales ~linearly with pixel noise for fixed geometry, so this
does one Monte Carlo run at a trial sigma and rescales. Run whenever the rig
geometry (camera poses) changes; paste the printed value into mocap_emulator.py's
PIXEL_NOISE_STD_PX.
"""
import numpy as np
import mujoco

from mocap_emulator import (MARKER_SITES, CAMERA_NAMES, MocapConfig,
                             _camera_projection, _project, _triangulate)

TARGET_RMS_M = 0.15e-3
TRIAL_SIGMA_PX = 1.0
N_TRIALS = 4000

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.mj_forward(model, data)

cfg = MocapConfig()
cam_ids = [model.camera(n).id for n in CAMERA_NAMES]
cams = [_camera_projection(model, data, cid, cfg.resolution, cfg.fovy_deg) for cid in cam_ids]
marker_ids = [model.site(n).id for n in MARKER_SITES]

rng = np.random.default_rng(0)
errs = []
for name, sid in zip(MARKER_SITES, marker_ids):
    X_true = data.site_xpos[sid].copy()
    uv = []
    for K, R_cv, cam_pos in cams:
        u, v, front = _project(K, R_cv, cam_pos, X_true)
        assert front, f"{name} not in front of a camera at home pose"
        uv.append((u, v))
    for _ in range(N_TRIALS):
        dets = [(K, R_cv, cam_pos, u + rng.normal(0, TRIAL_SIGMA_PX),
                 v + rng.normal(0, TRIAL_SIGMA_PX))
                for (K, R_cv, cam_pos), (u, v) in zip(cams, uv)]
        X_est = _triangulate(dets)
        errs.append(np.linalg.norm(X_est - X_true))

errs = np.array(errs)
rms_at_trial = np.sqrt(np.mean(errs ** 2))
scale = TARGET_RMS_M / rms_at_trial
calibrated_sigma = TRIAL_SIGMA_PX * scale

print(f"trial sigma = {TRIAL_SIGMA_PX} px  ->  3D RMS = {rms_at_trial*1000:.4f} mm "
      f"over {len(errs)} samples (4 markers x {N_TRIALS} trials, 3 cameras)")
print(f"target 3D RMS = {TARGET_RMS_M*1000:.2f} mm")
print(f"=> calibrated PIXEL_NOISE_STD_PX = {calibrated_sigma:.4f} px")

# sanity check at the calibrated sigma
errs2 = []
for name, sid in zip(MARKER_SITES, marker_ids):
    X_true = data.site_xpos[sid].copy()
    uv = []
    for K, R_cv, cam_pos in cams:
        u, v, _ = _project(K, R_cv, cam_pos, X_true)
        uv.append((u, v))
    for _ in range(N_TRIALS):
        dets = [(K, R_cv, cam_pos, u + rng.normal(0, calibrated_sigma),
                 v + rng.normal(0, calibrated_sigma))
                for (K, R_cv, cam_pos), (u, v) in zip(cams, uv)]
        X_est = _triangulate(dets)
        errs2.append(np.linalg.norm(X_est - X_true))
errs2 = np.array(errs2)
print(f"verification at calibrated sigma: 3D RMS = {np.sqrt(np.mean(errs2**2))*1000:.4f} mm")
