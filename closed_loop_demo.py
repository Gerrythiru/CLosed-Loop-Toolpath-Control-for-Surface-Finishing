"""Phase F: close the loop -- feed the mocap pose estimate back to correct the
toolpath, and check whether it actually helps.

The pre-planned serpentine path is converted once into the plate's own LOCAL
frame (toolpath_lib.to_local_frame), then re-projected into world space every
control tick using a HELD plate-pose estimate (toolpath_lib.apply_pose):

    world_target = apply_pose(held_pos, held_quat, local_traj[i])

`held_pos/held_quat` only update from a mocap sample once `t_sim` reaches the
sample's `t_available` (respects the modeled latency), only if `pose_valid`
(dropout holds the last value indefinitely -- no reaction to a known-bad
sample), and only within a rate limit (protects the controller from acting on
a single bad estimate that slipped past phase D's own checks -- independent of,
and in addition to, the temporal gate inside mocap_emulator.py, which protects
the *sensor estimate* rather than the *controller*).

Runs the SAME simulation three ways and plots the true tool-to-plate contact
error (distance from the tool tip to where the plan says it should be, measured
in the plate's TRUE/ground-truth current frame, not the estimate) for each:

  - open-loop:   held pose pinned to nominal (today's pre-phase-F behavior)
  - closed-loop: held pose driven by the real mocap estimate (latency + noise
                 + dropout + rate limit, all as modeled in phases D/E)
  - oracle:      held pose = the TRUE plate pose every physics step, no mocap
                 involved at all -- the best any correction scheme could
                 possibly do. Included because closing the loop turned out to
                 land close to this ceiling: the disturbance's per-tick jitter
                 is faster than the arm's control loop can track regardless of
                 sensing quality, so this is what proves the mocap chain isn't
                 the bottleneck (see mocap_rig_spec.md's "Phase F" section).

Run:
    python closed_loop_demo.py                # headless: writes CSV/plot for all 3 modes
    python closed_loop_demo.py --view          # live viewer, closed-loop mode, loops the path
    python closed_loop_demo.py --view --mode=open    # or --mode=oracle
    python closed_loop_demo.py --view --seed=7       # different disturbance schedule
                                                      # (defaults to 2; any non-negative int works)
"""
import csv
import sys
import time
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from toolpath_lib import (build_serpentine_knots, densify, ArmIK, settle_to_start,
                           to_local_frame, apply_pose)
from mocap_emulator import MocapEmulator, MocapScheduler, MocapConfig
from plate_disturbance import PlateDisturbance

MARGIN, N_PASSES, SKIM, APPROACH = 0.025, 7, 0.001, 0.06
FEED, PLUNGE, SUBSTEPS = 0.10, 0.04, 10
LOOKAHEAD = 4

# Controller-side rate limiter on the HELD pose (separate from, and in addition
# to, mocap_emulator.py's own per-marker temporal gate). Calibrated the same
# way as everything else in this project: measured genuine plate-origin motion
# under the actual disturbance (see mocap_rig_spec.md) -- p99.9 ~11.6 mm,
# max ~13.0 mm per 240 Hz tick -- and set ~25% above that.
CORR_RATE_BASE_M = 3e-3       # minimum allowed held-pose jump regardless of gap
CORR_RATE_SPEED_MPS = 3.1     # + this much allowance per second since last update
CORR_RATE_CAP_M = 0.09        # hard ceiling: ~2*sqrt(2)*30mm x/y envelope round trip

MODES = ("open", "closed", "oracle")


def _yaw_deg(quat, ref_quat=None):
    """Yaw-Euler-angle of `quat`, optionally relative to a reference
    orientation `ref_quat` (e.g. to express it "as seen from" the robot-base
    or plate-nominal frame). Exact for a pure Z-axis rotation (true here, by
    construction), a close approximation otherwise (e.g. mocap noise)."""
    if ref_quat is not None:
        rel = np.zeros(4)
        mujoco.mju_mulQuat(rel, np.array([ref_quat[0], -ref_quat[1], -ref_quat[2], -ref_quat[3]]), quat)
        quat = rel
    w, x, y, z = quat
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def run(mode, dist_seed=2, mocap_seed=0):
    """One full run. mode="open" reproduces today's pre-phase-F behavior exactly
    (held pose never leaves nominal) using the identical code path as "closed"."""
    assert mode in MODES
    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)
    plate_id = model.body("plate").id
    base_id = model.body("link_base").id
    tcp = model.site("tcp").id

    ik = ArmIK(model)
    qadr, dadr = ik.qadr, ik.dadr

    knots, corners, plate_top, pc, _ = build_serpentine_knots(
        model, margin=MARGIN, n_passes=N_PASSES, skim=SKIM, approach=APPROACH)
    CTRL_DT = model.opt.timestep * SUBSTEPS
    traj = densify(knots, CTRL_DT, feed=FEED, plunge=PLUNGE)

    dist = PlateDisturbance(model, seed=dist_seed)
    mocap = MocapEmulator(model, MocapConfig(seed=mocap_seed))
    sched = MocapScheduler(mocap)

    local_traj = to_local_frame(traj, dist.nominal_pos, dist.nominal_quat)

    held_pos, held_quat = dist.nominal_pos.copy(), dist.nominal_quat.copy()
    last_held_t = 0.0
    pending = []   # mocap samples generated but not yet "available" (latency)

    q_des = settle_to_start(model, data, ik, traj[0], qadr, dadr)

    log_t, log_contact_err = [], []
    log_true_disp, log_held_disp = [], []
    log_true_base, log_held_base = [], []        # pose (pos only) in robot-base frame, mm
    log_true_nominal, log_held_nominal = [], []  # pose (pos only) in plate-start frame, mm
    n_rate_limited = 0

    t_sim = 0.0
    for i, target in enumerate(traj):
        ff_local = local_traj[min(i + LOOKAHEAD, len(traj) - 1)]
        ff_world = apply_pose(held_pos, held_quat, ff_local)   # reacts to latest estimate only
        q_des = ik.solve(ff_world, q_des, iters=12)
        data.ctrl[:] = q_des
        for _ in range(SUBSTEPS):
            dist.apply(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            t_sim += model.opt.timestep

            if mode == "oracle":
                held_pos = data.xpos[plate_id].copy()
                held_quat = data.xquat[plate_id].copy()
            elif mode == "closed":
                s = sched.maybe_sample(data, t_sim)
                while s is not None:
                    pending.append(s)
                    s = sched.maybe_sample(data, t_sim)
                while pending and pending[0].t_available <= t_sim:
                    samp = pending.pop(0)
                    if not samp.pose_valid:
                        continue   # dropout: hold last known-good indefinitely
                    dt_gap = max(t_sim - last_held_t, 1e-6)
                    limit = min(CORR_RATE_BASE_M + CORR_RATE_SPEED_MPS * dt_gap,
                                CORR_RATE_CAP_M)
                    jump = np.linalg.norm(samp.pose_pos - held_pos)
                    if jump > limit:
                        n_rate_limited += 1
                        continue   # reject this update, keep holding
                    held_pos, held_quat = samp.pose_pos, samp.pose_quat
                    last_held_t = t_sim
            # mode == "open": held_pos/held_quat never change from nominal

        # ---- logging (once per control tick) ---------------------------------
        true_pos = data.xpos[plate_id].copy()
        true_quat = data.xquat[plate_id].copy()
        target_true_world = apply_pose(true_pos, true_quat, local_traj[i])
        contact_err = np.linalg.norm(data.site_xpos[tcp] - target_true_world) * 1000

        base_pos = data.xpos[base_id].copy()
        base_quat = np.zeros(4)
        mujoco.mju_mat2Quat(base_quat, data.xmat[base_id])

        # the disturbance is pure yaw about the plate's own origin (no
        # translation at all), so "displacement" is reported as yaw angle:
        # absolute in world frame, relative to each frame's own reference
        # orientation otherwise
        log_t.append(t_sim)
        log_contact_err.append(contact_err)
        log_true_disp.append(_yaw_deg(true_quat))
        log_held_disp.append(_yaw_deg(held_quat))
        log_true_base.append(_yaw_deg(true_quat, base_quat))
        log_held_base.append(_yaw_deg(held_quat, base_quat))
        log_true_nominal.append(_yaw_deg(true_quat, dist.nominal_quat))
        log_held_nominal.append(_yaw_deg(held_quat, dist.nominal_quat))

        if i % 400 == 0:
            print(f"  [{mode:6s}] tick {i}/{len(traj)}", flush=True)

    return dict(
        t=np.array(log_t), contact_err=np.array(log_contact_err),
        true_disp=np.array(log_true_disp), held_disp=np.array(log_held_disp),
        true_base=np.array(log_true_base), held_base=np.array(log_held_base),
        true_nominal=np.array(log_true_nominal), held_nominal=np.array(log_held_nominal),
        n_rate_limited=n_rate_limited,
    )


def _add_trail(scn, pts, rgba, width=0.0016):
    """Draw a point history as a growing poly-line in the render scene (same
    helper as trace_toolpath.py's, duplicated locally to keep each demo script
    self-contained)."""
    if len(pts) < 2:
        return
    P = np.asarray(pts, dtype=float)
    if len(P) > 160:
        P = P[np.linspace(0, len(P) - 1, 161).astype(int)]
    col = np.array(rgba, dtype=np.float32)
    for a, b in zip(P[:-1], P[1:]):
        if scn.ngeom >= scn.maxgeom:
            break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                            np.zeros(3), np.zeros(3), np.eye(3).ravel(), col)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)
        scn.ngeom += 1


def view(mode, dist_seed=2):
    """Live interactive viewer for one mode, looping the path. Mirrors run()'s
    setup and per-tick held-pose logic; see run() for the documented version."""
    import mujoco.viewer
    assert mode in MODES
    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)
    plate_id = model.body("plate").id
    tcp = model.site("tcp").id
    corner_site = model.site("mk_front_left").id   # traces a visible arc as the plate yaws

    ik = ArmIK(model)
    qadr, dadr = ik.qadr, ik.dadr

    knots, corners, plate_top, pc, _ = build_serpentine_knots(
        model, margin=MARGIN, n_passes=N_PASSES, skim=SKIM, approach=APPROACH)
    CTRL_DT = model.opt.timestep * SUBSTEPS
    traj = densify(knots, CTRL_DT, feed=FEED, plunge=PLUNGE)

    dist = PlateDisturbance(model, seed=dist_seed)
    mocap = MocapEmulator(model, MocapConfig(seed=0))
    sched = MocapScheduler(mocap)
    local_traj = to_local_frame(traj, dist.nominal_pos, dist.nominal_quat)

    held_pos, held_quat = dist.nominal_pos.copy(), dist.nominal_quat.copy()
    last_held_t = 0.0
    pending = []

    q_des = settle_to_start(model, data, ik, traj[0], qadr, dadr)

    print(f"opening interactive viewer in '{mode}' mode (disturbance seed={dist_seed}) - "
          f"orbit with the mouse, close the window to quit", flush=True)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as v:
        v.cam.lookat[:] = [pc[0], 0, plate_top + 0.02]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.25, 150, -32
        tool_trail, plate_trail = [], []
        t_sim = 0.0
        while v.is_running():
            for i, target in enumerate(traj):
                if not v.is_running():
                    break
                ff_local = local_traj[min(i + LOOKAHEAD, len(traj) - 1)]
                ff_world = apply_pose(held_pos, held_quat, ff_local)
                q_des = ik.solve(ff_world, q_des, iters=12)
                data.ctrl[:] = q_des
                for _ in range(SUBSTEPS):
                    dist.apply(data, model.opt.timestep)
                    mujoco.mj_step(model, data)
                    t_sim += model.opt.timestep

                    if mode == "oracle":
                        held_pos = data.xpos[plate_id].copy()
                        held_quat = data.xquat[plate_id].copy()
                    elif mode == "closed":
                        s = sched.maybe_sample(data, t_sim)
                        while s is not None:
                            pending.append(s)
                            s = sched.maybe_sample(data, t_sim)
                        while pending and pending[0].t_available <= t_sim:
                            samp = pending.pop(0)
                            if not samp.pose_valid:
                                continue
                            dt_gap = max(t_sim - last_held_t, 1e-6)
                            limit = min(CORR_RATE_BASE_M + CORR_RATE_SPEED_MPS * dt_gap,
                                        CORR_RATE_CAP_M)
                            if np.linalg.norm(samp.pose_pos - held_pos) > limit:
                                continue
                            held_pos, held_quat = samp.pose_pos, samp.pose_quat
                            last_held_t = t_sim
                    # mode == "open": held pose never leaves nominal

                tool_trail.append(data.site_xpos[tcp].copy())
                plate_trail.append(data.site_xpos[corner_site].copy())
                v.user_scn.ngeom = 0
                _add_trail(v.user_scn, tool_trail, (1.0, 0.1, 0.1, 1.0))    # red: tool
                _add_trail(v.user_scn, plate_trail, (0.1, 0.4, 1.0, 1.0))   # blue: plate corner arc
                v.sync()
                time.sleep(CTRL_DT)
            tool_trail.clear()
            plate_trail.clear()


def main(dist_seed=2):
    runs = {}
    for mode in MODES:
        print(f"running {mode} pass... (disturbance seed={dist_seed})")
        runs[mode] = run(mode, dist_seed=dist_seed)

    # ---------------------------------------------------------------------- summary
    print("-" * 72)
    for mode in MODES:
        e = runs[mode]["contact_err"]
        print(f"{mode:12s} tool-to-true-plate contact error: mean {e.mean():.2f} mm, "
              f"RMS {np.sqrt(np.mean(e ** 2)):.2f} mm, max {e.max():.2f} mm")
    print(f"closed: {runs['closed']['n_rate_limited']} controller-side updates rejected "
          f"by the rate limiter")
    oracle_mean = runs["oracle"]["contact_err"].mean()
    closed_mean = runs["closed"]["contact_err"].mean()
    open_mean = runs["open"]["contact_err"].mean()
    headroom = (closed_mean - oracle_mean) / max(open_mean - oracle_mean, 1e-9)
    print(f"closed-loop closes {100 * (1 - headroom):.0f}% of the open->oracle gap "
          f"(mocap chain is within {100 * (closed_mean / oracle_mean - 1):.1f}% of the "
          f"perfect-sensor ceiling)")
    print("-" * 72)

    # ---------------------------------------------------------------------- outputs
    with open("closed_loop_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "mode", "contact_err_mm",
                    "true_yaw_world_deg", "held_yaw_world_deg",
                    "true_yaw_base_deg", "held_yaw_base_deg",
                    "true_yaw_nominal_deg", "held_yaw_nominal_deg"])
        for mode in MODES:
            r = runs[mode]
            for row in zip(r["t"], [mode] * len(r["t"]), r["contact_err"],
                            r["true_disp"], r["held_disp"],
                            r["true_base"], r["held_base"],
                            r["true_nominal"], r["held_nominal"]):
                w.writerow(row)
    print("wrote closed_loop_log.csv (world / robot-base / plate-start frames)")

    colors = {"open": "tab:red", "closed": "tab:green", "oracle": "tab:blue"}
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for mode in MODES:
        r = runs[mode]
        ax[0].plot(r["t"], r["contact_err"], color=colors[mode], lw=0.8, label=mode)
    ax[0].set_ylabel("tool-to-true-plate\ncontact error [mm]")
    ax[0].set_title("Phase F: does closing the loop help? (vs. TRUE plate pose, not the estimate)")
    ax[0].legend(fontsize=8)

    ax[1].plot(runs["open"]["t"], runs["open"]["true_disp"], color="black", lw=0.9,
               label="true plate yaw")
    ax[1].plot(runs["closed"]["t"], runs["closed"]["held_disp"], color="tab:green", lw=0.7,
               ls="--", label="closed-loop: held estimate")
    ax[1].plot(runs["oracle"]["t"], runs["oracle"]["held_disp"], color="tab:blue", lw=0.7,
               ls=":", label="oracle: held (=true)")
    ax[1].set_ylabel("yaw from\nnominal [deg]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(fontsize=8)

    fig.tight_layout()
    fig.savefig("closed_loop_result.png", dpi=110)
    print("wrote closed_loop_result.png")


if __name__ == "__main__":
    view_mode = "closed"
    dist_seed = 2
    for arg in sys.argv:
        if arg.startswith("--mode="):
            view_mode = arg.split("=", 1)[1]
        elif arg.startswith("--seed="):
            dist_seed = int(arg.split("=", 1)[1])

    if "--view" in sys.argv:
        view(view_mode, dist_seed=dist_seed)
    else:
        main(dist_seed=dist_seed)
