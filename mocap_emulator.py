"""Phase D: emulated OptiTrack PrimeX 22 mocap — noise, occlusion, and failure modes.

Pipeline per mocap sample (matches the checklist agreed for phase D):

  true 3D marker positions
    -> project into each camera's pixel plane (pinhole, from fovy + resolution)
    -> occlusion test (mj_ray, camera -> marker, against arm/plate/table)      [dropout]
    -> marker-swap check (two markers close together in one camera's image)   [label flips]
    -> ghost-detection injection (occluded marker "seen" via a false reflection) [ghosts]
    -> per-camera 2D centroid noise (sub-pixel Gaussian)                      [2D noise]
    -> multi-view DLT triangulation per marker (>=2 cameras required)         [3D noise]
    -> global calibration bias + slow drift, applied to all markers           [bias/drift]
    -> per-marker temporal plausibility gate: reject a marker whose jump since
       its own last accepted position is implausibly large for the elapsed time
       -- runs independently of, and BEFORE, the spatial check below, so it can
       still catch a bad marker even when only 3 (no spare) are available     [temporal gate]
    -> Kabsch rigid-body fit (>=3 of 4 markers required) -> plate pose estimate
    -> residual check: refit without any marker whose fit residual is an outlier
       (only possible with 4 good markers -- there's no spare point to give up
       once down to the minimum of 3)                                        [outlier rejection]
    -> timestamped with the sample time and the time it becomes available (+latency)

Reuses the rig constants from mocap_rig_spec.md. See calibrate_mocap_noise.py for
how PIXEL_NOISE_STD_PX below was chosen to hit the ~0.15 mm 3D RMS target.
"""
from dataclasses import dataclass, field
import numpy as np
import mujoco

MARKER_SITES = ["mk_front_left", "mk_front_right", "mk_back_left", "mk_back_right"]
CAMERA_NAMES = ["cam_overhead", "cam_oblique_w", "cam_oblique_e"]

# Calibrated against the actual rig geometry by calibrate_mocap_noise.py so that,
# with all 3 cameras unoccluded, triangulated 3D marker noise has ~0.15 mm RMS
# (the PrimeX 22's quoted calibrated-volume accuracy). Verified result: 0.1498 mm.
PIXEL_NOISE_STD_PX = 0.1014


@dataclass
class MocapConfig:
    rate_hz: float = 240.0             # emulated output rate
    latency_s: float = 0.008           # end-to-end (camera + streaming) latency
    pixel_noise_std_px: float = PIXEL_NOISE_STD_PX
    resolution: tuple = (2048, 1088)   # PrimeX 22 sensor, px
    fovy_deg: float = 47.0             # PrimeX 22 vertical FOV, deg (see mocap_rig_spec.md)
    calib_bias_std_m: float = 0.10e-3   # fixed per-run calibration offset, magnitude
    drift_amplitude_m: float = 0.05e-3  # slow-drift amplitude
    drift_period_s: float = 90.0        # slow-drift period
    swap_pixel_threshold_px: float = 3.0   # markers closer than this in-frame may swap
    swap_prob: float = 0.30                # probability of a swap when threshold is met
    ghost_prob_per_cam_frame: float = 0.01   # per camera, per frame, per occluded marker
    ghost_pixel_jitter_px: float = 15.0      # ghost detections are noisier than real ones
    min_cams_per_marker: int = 2       # rigid-body-solve requirement (per the rig spec)
    min_markers_per_pose: int = 3      # ditto
    outlier_residual_m: float = 2e-3   # Kabsch fit residual above this = reject the marker
    temporal_gate_enabled: bool = True
    temporal_base_m: float = 3e-3           # minimum allowed per-marker jump regardless of gap
    temporal_speed_mps: float = 3.45        # + this much allowance per second since last seen
    # (base+speed calibrated so the limit at the nominal 240 Hz tick, ~17.4 mm, sits
    #  ~25% above the worst genuine per-tick marker motion seen at the +/-30 mm / 2 Hz
    #  disturbance bound -- see the "Temporal gate calibration" note in mocap_rig_spec.md.
    #  Recalibrate if TRANSLATION_BOUND_M or BANDWIDTH_HZ change.)
    temporal_envelope_cap_m: float = 0.11   # hard ceiling on the allowance, however long the gap
    # (base+speed*dt_gap grows without bound for a long-missing marker, which is wrong: once
    #  the gap exceeds a few disturbance correlation times (~0.08 s here) the marker has fully
    #  decorrelated from its old position, so "how long has it been gone" stops being useful
    #  information -- the only honest ceiling left is "still somewhere in the known envelope."
    #  0.11 m is just above the +/-30 mm bound's worst-case 3-axis round trip
    #  (2*sqrt(3)*30mm = ~103.9 mm), so it never clips even the most extreme legitimate
    #  reappearance. Revisit if TRANSLATION_BOUND_M changes.)
    seed: int = 0


def _camera_projection(model, data, cam_id, resolution, fovy_deg):
    """Pinhole intrinsics/extrinsics for a MuJoCo camera.

    Returns (K, R_cv, t, cam_pos): world point X projects as
        p_cv = R_cv @ (X - cam_pos);  [u,v,1] ~ K @ p_cv   (valid iff p_cv[2] > 0)
    R_cv is the world->camera rotation in a right/down/forward (CV) frame.
    """
    W, H = resolution
    fy = (H / 2.0) / np.tan(np.radians(fovy_deg) / 2.0)
    fx = fy
    cx, cy = W / 2.0, H / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    cam_pos = data.cam_xpos[cam_id].copy()
    cam_mat = data.cam_xmat[cam_id].reshape(3, 3)   # columns = camera local axes in world
    x_cv = cam_mat[:, 0]        # local +x (right) stays right
    y_cv = -cam_mat[:, 1]       # local +y (up) -> image +v is down
    z_cv = -cam_mat[:, 2]       # local -z (view direction) -> +forward
    R_cv = np.stack([x_cv, y_cv, z_cv], axis=0)     # world -> camera
    return K, R_cv, cam_pos


def _project(K, R_cv, cam_pos, X):
    """World point -> (u, v, in_front). in_front=False means behind the camera."""
    p = R_cv @ (X - cam_pos)
    if p[2] <= 1e-6:
        return None, None, False
    uvw = K @ p
    return uvw[0] / uvw[2], uvw[1] / uvw[2], True


def _in_frame(u, v, resolution):
    return 0 <= u < resolution[0] and 0 <= v < resolution[1]


def _triangulate(dets):
    """Linear DLT triangulation from >=2 (K, R_cv, cam_pos, u, v) detections."""
    A = []
    for K, R_cv, cam_pos, u, v in dets:
        P = K @ np.hstack([R_cv, -(R_cv @ cam_pos)[:, None]])   # 3x4 camera matrix
        A.append(u * P[2] - P[0])
        A.append(v * P[2] - P[1])
    A = np.array(A)
    _, _, Vt = np.linalg.svd(A)
    Xh = Vt[-1]
    return Xh[:3] / Xh[3]


def _kabsch(local_pts, world_pts):
    """Best-fit rigid transform (R, t) with world = R @ local + t."""
    lc = local_pts - local_pts.mean(axis=0)
    wc = world_pts - world_pts.mean(axis=0)
    H = lc.T @ wc
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = world_pts.mean(axis=0) - R @ local_pts.mean(axis=0)
    return R, t


def _fit_rigid_robust(local_pts, world_pts, names, min_markers, outlier_threshold_m):
    """Kabsch fit with one round of residual-based outlier rejection per spare point.

    Fits all given points, then checks each one's residual (distance between its
    reconstructed position and where the fitted rigid transform says it should
    be). If there's a spare point above `min_markers` and the worst residual
    exceeds `outlier_threshold_m`, that marker is dropped and the fit redone with
    the rest -- repeating until either no residual is over threshold or dropping
    another marker would go below `min_markers` (with only 4 markers total,
    that's at most one rejection: 4 -> 3).

    Returns (R, t, used_names, rejected, residuals):
      used_names  -- markers actually in the final fit
      rejected    -- [(name, residual_m), ...] for each marker dropped, in order
      residuals   -- {name: residual_m} for the final fit's surviving markers
    """
    idx = list(range(len(names)))
    rejected = []
    while True:
        L, Wp = local_pts[idx], world_pts[idx]
        R, t = _kabsch(L, Wp)
        fitted = (R @ L.T).T + t
        resid = np.linalg.norm(fitted - Wp, axis=1)
        worst = int(np.argmax(resid))
        if len(idx) > min_markers and resid[worst] > outlier_threshold_m:
            rejected.append((names[idx[worst]], float(resid[worst])))
            idx.pop(worst)
            continue
        residuals = {names[i]: float(r) for i, r in zip(idx, resid)}
        return R, t, [names[i] for i in idx], rejected, residuals


@dataclass
class MocapSample:
    t_capture: float
    t_available: float
    n_markers_valid: int
    marker_valid: dict            # name -> bool
    marker_world: dict            # name -> reconstructed 3D pos (or None)
    marker_cams_used: dict        # name -> list of camera names that contributed
    pose_valid: bool
    pose_pos: np.ndarray          # plate body-frame origin estimate (world), or None
    pose_quat: np.ndarray         # or None
    pose_markers_used: list       # marker names that actually went into the final fit
    outlier_rejected: list        # [(name, residual_m), ...] dropped by the residual check
    fit_residuals: dict           # name -> residual (m) for markers in the final fit
    temporal_rejected: list       # [(name, jump_m), ...] dropped by the temporal gate
    events: list                  # human-readable strings: swaps, ghosts, dropouts, rejections


class MocapEmulator:
    """Stateful emulator: call `sample(t)` once per scheduled mocap tick."""

    def __init__(self, model, config: MocapConfig = None):
        self.model = model
        self.cfg = config or MocapConfig()
        self.rng = np.random.default_rng(self.cfg.seed)

        self.marker_site_ids = [model.site(n).id for n in MARKER_SITES]
        self.cam_ids = [model.camera(n).id for n in CAMERA_NAMES]

        plate_id = model.body("plate").id
        # local (plate-frame) marker offsets, needed for the rigid-body fit
        self.local_offsets = np.array([
            model.site_pos[sid] for sid in self.marker_site_ids
        ])
        assert all(model.site_bodyid[sid] == plate_id for sid in self.marker_site_ids)

        # fixed-per-run calibration bias direction + slow-drift direction
        d1 = self.rng.normal(size=3); self.bias_dir = d1 / np.linalg.norm(d1)
        d2 = self.rng.normal(size=3); self.drift_dir = d2 / np.linalg.norm(d2)
        self.drift_phase = self.rng.uniform(0, 2 * np.pi)

        # per-marker temporal gate state: last accepted position/time, independent
        # of the spatial (Kabsch residual) check
        self._last_good_pos = {name: None for name in MARKER_SITES}
        self._last_good_t = {name: None for name in MARKER_SITES}

        self._raycast_geomid = np.zeros(1, dtype=np.int32)
        # mj_ray ignores contype/conaffinity and only respects this group mask; the
        # camera housings (group 4) sit right next to their own camera's optical
        # center and must never self-occlude it. Every other group (table, plate,
        # arm, markers, truss) stays included.
        self._raycast_geomgroup = np.array([1, 1, 1, 1, 0, 1], dtype=np.uint8)

    def _calib_offset(self, t):
        bias = self.cfg.calib_bias_std_m * self.bias_dir
        drift = (self.cfg.drift_amplitude_m
                 * np.sin(2 * np.pi * t / self.cfg.drift_period_s + self.drift_phase)
                 * self.drift_dir)
        return bias + drift

    def _occluded(self, data, cam_pos, target, exclude_geomid):
        """True if something other than the marker itself blocks the line of sight."""
        vec = target - cam_pos
        dist = np.linalg.norm(vec)
        if dist < 1e-9:
            return False
        vec = vec / dist
        hit_dist = mujoco.mj_ray(self.model, data, cam_pos, vec, self._raycast_geomgroup,
                                  1, -1, self._raycast_geomid)
        if hit_dist < 0:
            return True   # nothing hit at all -- shouldn't happen, treat as occluded
        # unoccluded if the nearest hit is (approximately) the marker itself
        return not (self._raycast_geomid[0] == exclude_geomid or hit_dist >= dist - 3e-3)

    def sample(self, data, t):
        """Take one mocap sample from the current MuJoCo state `data` at sim time `t`."""
        cfg = self.cfg
        cams = [_camera_projection(self.model, data, cid, cfg.resolution, cfg.fovy_deg)
                for cid in self.cam_ids]

        marker_geom_ids = [self.model.geom(f"{n}_geom").id for n in MARKER_SITES]
        true_world = {n: data.site_xpos[sid].copy()
                      for n, sid in zip(MARKER_SITES, self.marker_site_ids)}

        # ---- per-camera raw visibility + pixel projection --------------------------
        # detections[cam_idx][marker_name] = (u, v, is_ghost)
        detections = [dict() for _ in cams]
        raw_visible = [dict() for _ in cams]   # marker -> (u, v) if geometrically visible
        events = []

        for ci, (K, R_cv, cam_pos) in enumerate(cams):
            for mi, name in enumerate(MARKER_SITES):
                X = true_world[name]
                u, v, front = _project(K, R_cv, cam_pos, X)
                if not front or not _in_frame(u, v, cfg.resolution):
                    continue
                if self._occluded(data, cam_pos, X, marker_geom_ids[mi]):
                    continue
                raw_visible[ci][name] = (u, v)

        # ---- marker-swap: two markers close together in one camera's image ---------
        for ci in range(len(cams)):
            names = list(raw_visible[ci].keys())
            for a in range(len(names)):
                for b in range(a + 1, len(names)):
                    na, nb = names[a], names[b]
                    ua, va = raw_visible[ci][na]
                    ub, vb = raw_visible[ci][nb]
                    if np.hypot(ua - ub, va - vb) < cfg.swap_pixel_threshold_px:
                        if self.rng.uniform() < cfg.swap_prob:
                            raw_visible[ci][na], raw_visible[ci][nb] = (ub, vb), (ua, va)
                            events.append(f"t={t:.3f} {CAMERA_NAMES[ci]}: "
                                          f"label swap {na}<->{nb}")

            for name, (u, v) in raw_visible[ci].items():
                detections[ci][name] = (u, v, False)

        # ---- ghost detections: an occluded marker "reappears" via a reflection -----
        for ci, (K, R_cv, cam_pos) in enumerate(cams):
            for mi, name in enumerate(MARKER_SITES):
                if name in detections[ci]:
                    continue
                if self.rng.uniform() >= cfg.ghost_prob_per_cam_frame:
                    continue
                X = true_world[name]
                u, v, front = _project(K, R_cv, cam_pos, X)
                if not front:
                    continue
                u += self.rng.normal(0, cfg.ghost_pixel_jitter_px)
                v += self.rng.normal(0, cfg.ghost_pixel_jitter_px)
                if not _in_frame(u, v, cfg.resolution):
                    continue
                detections[ci][name] = (u, v, True)
                events.append(f"t={t:.3f} {CAMERA_NAMES[ci]}: ghost detection for {name}")

        # ---- per-camera 2D centroid noise -------------------------------------------
        for ci in range(len(cams)):
            for name in detections[ci]:
                u, v, is_ghost = detections[ci][name]
                u += self.rng.normal(0, cfg.pixel_noise_std_px)
                v += self.rng.normal(0, cfg.pixel_noise_std_px)
                detections[ci][name] = (u, v, is_ghost)

        # ---- per-marker triangulation ------------------------------------------------
        calib = self._calib_offset(t)
        marker_valid, marker_world, marker_cams_used = {}, {}, {}
        for name in MARKER_SITES:
            contributing = [ci for ci in range(len(cams)) if name in detections[ci]]
            if len(contributing) < cfg.min_cams_per_marker:
                marker_valid[name] = False
                marker_world[name] = None
                marker_cams_used[name] = [CAMERA_NAMES[ci] for ci in contributing]
                events.append(f"t={t:.3f} {name}: dropout "
                               f"({len(contributing)}/{cfg.min_cams_per_marker} cams)")
                continue
            dets = [(cams[ci][0], cams[ci][1], cams[ci][2],
                     detections[ci][name][0], detections[ci][name][1])
                    for ci in contributing]
            X_est = _triangulate(dets) + calib
            marker_valid[name] = True
            marker_world[name] = X_est
            marker_cams_used[name] = [CAMERA_NAMES[ci] for ci in contributing]

        # ---- per-marker temporal plausibility gate ------------------------------------
        # Independent of the spatial (Kabsch) check below and runs first, so it can
        # catch a bad marker even when there's no 4th, spare marker to cross-check
        # against this frame. Compares each marker only to ITS OWN last accepted
        # position -- no other marker's data is involved.
        temporal_rejected = []
        if cfg.temporal_gate_enabled:
            for name in MARKER_SITES:
                if not marker_valid[name]:
                    continue
                last_pos, last_t = self._last_good_pos[name], self._last_good_t[name]
                if last_pos is not None:
                    dt_gap = max(t - last_t, 1e-6)
                    limit = min(cfg.temporal_base_m + cfg.temporal_speed_mps * dt_gap,
                                cfg.temporal_envelope_cap_m)
                    jump = np.linalg.norm(marker_world[name] - last_pos)
                    if jump > limit:
                        temporal_rejected.append((name, float(jump)))
                        events.append(f"t={t:.3f} {name}: rejected by temporal gate "
                                      f"(jump {jump*1000:.2f} mm over {dt_gap*1000:.1f} ms, "
                                      f"limit {limit*1000:.2f} mm)")
                        marker_valid[name] = False
                        marker_world[name] = None
                        continue
                self._last_good_pos[name] = marker_world[name]
                self._last_good_t[name] = t

        # ---- rigid-body fit, with residual-based outlier rejection --------------------
        good = [i for i, n in enumerate(MARKER_SITES) if marker_valid[n]]
        pose_valid = False
        pose_pos = pose_quat = None
        pose_markers_used, outlier_rejected, fit_residuals = [], [], {}
        if len(good) >= cfg.min_markers_per_pose:
            L = self.local_offsets[good]
            Wp = np.array([marker_world[MARKER_SITES[i]] for i in good])
            names = [MARKER_SITES[i] for i in good]
            R, tvec, pose_markers_used, outlier_rejected, fit_residuals = _fit_rigid_robust(
                L, Wp, names, cfg.min_markers_per_pose, cfg.outlier_residual_m)
            for rn, resid_m in outlier_rejected:
                events.append(f"t={t:.3f} {rn}: rejected as fit outlier "
                               f"(residual {resid_m*1000:.2f} mm)")
            pose_valid = len(pose_markers_used) >= cfg.min_markers_per_pose
            if pose_valid:
                pose_pos = tvec
                pose_quat = np.zeros(4)
                mujoco.mju_mat2Quat(pose_quat, R.ravel())
        if not pose_valid:
            events.append(f"t={t:.3f} pose: DROPPED "
                           f"({len(good)}/{cfg.min_markers_per_pose} markers)")

        return MocapSample(
            t_capture=t, t_available=t + cfg.latency_s,
            n_markers_valid=len(good), marker_valid=marker_valid,
            marker_world=marker_world, marker_cams_used=marker_cams_used,
            pose_valid=pose_valid, pose_pos=pose_pos, pose_quat=pose_quat,
            pose_markers_used=pose_markers_used, outlier_rejected=outlier_rejected,
            fit_residuals=fit_residuals, temporal_rejected=temporal_rejected,
            events=events,
        )


class MocapScheduler:
    """Fires MocapEmulator.sample() at a fixed rate against an irregular sim clock."""

    def __init__(self, emulator: MocapEmulator, rate_hz=None):
        self.emulator = emulator
        self.period = 1.0 / (rate_hz or emulator.cfg.rate_hz)
        self.next_t = 0.0

    def maybe_sample(self, data, t_sim):
        """Call every physics/control step; returns a MocapSample or None."""
        if t_sim + 1e-9 < self.next_t:
            return None
        sample = self.emulator.sample(data, self.next_t)
        self.next_t += self.period
        return sample
