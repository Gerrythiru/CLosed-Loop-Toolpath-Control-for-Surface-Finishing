"""Closed-loop demo: the Lite 6 traces an ordered linear (serpentine) tool path
across the work plate.

- Build an ordered raster path over the plate surface.
- Densify it into a constant-feedrate Cartesian trajectory.
- Track it in closed loop: per control tick, 6-DOF damped-least-squares IK turns the
  Cartesian pose error into a joint-target increment; the joint targets are sent to
  the Lite 6 position actuators and the physics is stepped.
- Record commanded vs. actual TCP, write a GIF and a result plot.

Run:
    python trace_toolpath.py           # headless: writes toolpath_trace.gif + toolpath_result.png
    python trace_toolpath.py --view    # live interactive window (orbit/zoom with the mouse)
"""
import os
import sys
import time
import numpy as np
import mujoco
import PIL.Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from toolpath_lib import build_serpentine_knots, densify as _densify, ArmIK, settle_to_start

VIEW = "--view" in sys.argv

# ----------------------------------------------------------------------------- setup
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
tcp = model.site("tcp").id

ik = ArmIK(model)
qadr, dadr = ik.qadr, ik.dadr

# ------------------------------------------------------------------ ordered raster path
MARGIN = 0.025          # keep the tip this far inside the plate edge
N_PASSES = 7            # number of lines in the x direction
SKIM = 0.001           # trace this far above the surface
APPROACH = 0.06        # lead-in / lead-out height above the surface

path_knots, corners, plate_top, pc, (hx, hy, hz) = build_serpentine_knots(
    model, margin=MARGIN, n_passes=N_PASSES, skim=SKIM, approach=APPROACH)

# ------------------------------------------------------- densify to a feedrate trajectory
FEED = 0.10            # m/s along the path
PLUNGE = 0.04          # m/s for the vertical lead-in / lead-out
SUBSTEPS = 10                              # physics steps per control tick
CTRL_DT = model.opt.timestep * SUBSTEPS   # one IK/control tick per SUBSTEPS steps

def densify(knots):
    return _densify(knots, CTRL_DT, feed=FEED, plunge=PLUNGE)

traj = densify(path_knots)
path_len = np.sum(np.linalg.norm(np.diff(corners, axis=0), axis=1))
print(f"plate top z = {plate_top:.3f} m,  raster {N_PASSES} passes,  "
      f"path length on surface = {path_len:.2f} m")
print(f"trajectory points = {len(traj)},  sim time ~ {len(traj) * CTRL_DT:.1f} s")

def solve_ik(target_pos, q0, iters=80, tol=1e-4):
    return ik.solve(target_pos, q0, iters=iters, tol=tol)

# --- reach the start of the path, then let the physics settle --------------------
q_des = settle_to_start(model, data, ik, traj[0], qadr, dadr)
start_err = np.linalg.norm((traj[0] - data.site_xpos[tcp])) * 1000
print(f"start-pose error after settle: {start_err:.2f} mm", flush=True)

_TRAIL_MAX = 160          # cap on trail segments drawn per frame

def _add_trail(scn, pts, rgba=(1.0, 0.1, 0.1, 1.0), width=0.0016):
    """Draw the actual TCP history as a growing red poly-line in the render scene."""
    if len(pts) < 2:
        return
    P = np.asarray(pts, dtype=float)
    if len(P) > _TRAIL_MAX + 1:                      # decimate to keep it cheap
        idx = np.linspace(0, len(P) - 1, _TRAIL_MAX + 1).astype(int)
        P = P[idx]
    col = np.array(rgba, dtype=np.float32)
    for a, b in zip(P[:-1], P[1:]):
        if scn.ngeom >= scn.maxgeom:
            break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                            np.zeros(3), np.zeros(3), np.eye(3).ravel(), col)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)
        scn.ngeom += 1

GIF_EVERY = 8         # keep every Nth control tick as a frame
LOOKAHEAD = 4         # ticks of feed-forward to cancel the position-loop lag

# ------------------------------------------------------------ interactive viewer mode
if VIEW:
    import mujoco.viewer
    print("opening interactive viewer - orbit with the mouse, close the window to quit",
          flush=True)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as v:
        v.cam.lookat[:] = [pc[0], 0, plate_top + 0.02]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.25, 150, -32
        trail = []
        while v.is_running():
            for i, target in enumerate(traj):
                if not v.is_running():
                    break
                ff = traj[min(i + LOOKAHEAD, len(traj) - 1)]
                q_des = solve_ik(ff, q_des, iters=12)
                data.ctrl[:] = q_des
                for _ in range(SUBSTEPS):
                    mujoco.mj_step(model, data)
                trail.append(data.site_xpos[tcp].copy())
                v.user_scn.ngeom = 0
                _add_trail(v.user_scn, trail)
                v.sync()
                time.sleep(CTRL_DT)
            trail.clear()                       # loop the path again
    sys.exit(0)

# ------------------------------------------------------------------ headless recording
cmd_xyz, act_xyz = [], []
frames = []
ren = mujoco.Renderer(model, 384, 512)
cam = mujoco.MjvCamera()
cam.lookat[:] = [pc[0], 0, plate_top + 0.02]
cam.distance, cam.azimuth, cam.elevation = 1.15, 150, -35

for i, target in enumerate(traj):
    ff = traj[min(i + LOOKAHEAD, len(traj) - 1)]     # look-ahead setpoint
    q_des = solve_ik(ff, q_des, iters=12)            # warm-started, setpoint moves ~2 mm/tick
    data.ctrl[:] = q_des
    for _ in range(SUBSTEPS):
        mujoco.mj_step(model, data)

    cmd_xyz.append(target.copy())                    # log the on-path target, not the look-ahead
    act_xyz.append(data.site_xpos[tcp].copy())

    if i % 400 == 0:
        print(f"  tick {i}/{len(traj)}", flush=True)

    if i % GIF_EVERY == 0:
        ren.update_scene(data, cam)
        _add_trail(ren.scene, act_xyz)          # defined below
        frames.append(ren.render())

cmd_xyz = np.array(cmd_xyz)
act_xyz = np.array(act_xyz)

# on-surface tracking error (skip the vertical lead-in / lead-out portions)
on_surf = cmd_xyz[:, 2] < plate_top + APPROACH * 0.5
err_mm = np.linalg.norm(cmd_xyz - act_xyz, axis=1) * 1000
print(f"TCP tracking error (on surface):  mean {err_mm[on_surf].mean():.2f} mm   "
      f"max {err_mm[on_surf].max():.2f} mm")

gif = [PIL.Image.fromarray(f).resize((360, 270)).convert(
           "P", palette=PIL.Image.ADAPTIVE, colors=128)
       for f in frames]
gif[0].save("toolpath_trace.gif", save_all=True, append_images=gif[1:],
            duration=int(GIF_EVERY * CTRL_DT * 1000), loop=0, optimize=True)
print(f"wrote toolpath_trace.gif ({len(frames)} frames, "
      f"{os.path.getsize('toolpath_trace.gif') / 1e6:.1f} MB)")

# ------------------------------------------------------------------------------ plots
fig, ax = plt.subplots(1, 2, figsize=(11, 4.4))
ax[0].add_patch(plt.Rectangle((pc[0] - hx, pc[1] - hy), 2 * hx, 2 * hy,
                              fc="0.9", ec="0.5", label="plate"))
ax[0].plot(cmd_xyz[:, 0], cmd_xyz[:, 1], "-", lw=1.0, color="tab:blue", label="commanded")
ax[0].plot(act_xyz[:, 0], act_xyz[:, 1], "--", lw=1.0, color="tab:red", label="actual TCP")
ax[0].plot(cmd_xyz[0, 0], cmd_xyz[0, 1], "go", ms=6, label="start")
ax[0].plot(cmd_xyz[-1, 0], cmd_xyz[-1, 1], "ks", ms=6, label="end")
ax[0].set_aspect("equal"); ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
ax[0].set_title("Ordered serpentine tool path (top view)"); ax[0].legend(fontsize=8, loc="upper right")

t = np.arange(len(err_mm)) * CTRL_DT
ax[1].plot(t, err_mm, color="tab:red")
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("TCP position error [mm]")
ax[1].set_title("Closed-loop tracking error"); ax[1].grid(alpha=0.3)
fig.tight_layout(); fig.savefig("toolpath_result.png", dpi=110)
print("wrote toolpath_result.png")
