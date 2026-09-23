"""Shared path-planning + IK helpers for the Closed_Loop_ToolPath demos.

Factored out of trace_toolpath.py so mocap_demo.py (and anything else that needs
to drive the Lite 6 through the serpentine tool path) doesn't duplicate the path
construction / 6-DOF IK logic.
"""
import numpy as np
import mujoco


def _quat_to_mat(quat):
    """4-vector quaternion -> 3x3 rotation matrix."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
    return R.reshape(3, 3)


def to_local_frame(points, ref_pos, ref_quat):
    """World-frame point(s) -> the local frame of a body at (ref_pos, ref_quat).

    Inverse of `apply_pose`. `points` is (3,) or (N, 3); world = R @ local + t,
    so local = R^T @ (world - t) -- written here as row-vector matmuls so it
    works directly on an (N, 3) trajectory array.
    """
    R = _quat_to_mat(ref_quat)
    return (np.asarray(points) - ref_pos) @ R


def apply_pose(pos, quat, local_points):
    """Local-frame point(s) -> world frame, given a rigid pose (pos, quat).

    Inverse of `to_local_frame`, and the forward direction of the same
    convention used by the rigid-body fit in mocap_emulator.py
    (`_kabsch`/`_fit_rigid_robust`: world = R @ local + t) -- re-projecting a
    pre-planned local waypoint through a live pose estimate is exactly that
    fit run in reverse.
    """
    R = _quat_to_mat(quat)
    return np.asarray(local_points) @ R.T + pos


def build_serpentine_knots(model, plate_body="plate", plate_geom="plate_geom",
                            margin=0.025, n_passes=7, skim=0.001, approach=0.06):
    """Ordered raster (boustrophedon) knots over the plate, plus lead-in/lead-out.

    Returns (knots, corners, plate_top, plate_center_xy, plate_halfsize).
    """
    pc = model.body(plate_body).pos.copy()
    hx, hy, hz = model.geom(plate_geom).size
    plate_top = pc[2] + hz
    x0, x1 = pc[0] - hx + margin, pc[0] + hx - margin
    y0, y1 = pc[1] - hy + margin, pc[1] + hy - margin
    z = plate_top + skim

    corners = []
    for i, x in enumerate(np.linspace(x0, x1, n_passes)):
        ys = (y0, y1) if i % 2 == 0 else (y1, y0)
        corners.append([x, ys[0], z])
        corners.append([x, ys[1], z])
    corners = np.array(corners)

    knots = np.vstack([
        [corners[0, 0], corners[0, 1], plate_top + approach],
        corners,
        [corners[-1, 0], corners[-1, 1], plate_top + approach],
    ])
    return knots, corners, plate_top, pc, np.array([hx, hy, hz])


def densify(knots, ctrl_dt, feed=0.10, plunge=0.04):
    """Turn ordered knots into a constant-feedrate Cartesian trajectory."""
    pts = [knots[0]]
    for a, b in zip(knots[:-1], knots[1:]):
        seg = b - a
        L = np.linalg.norm(seg)
        speed = plunge if abs(seg[2]) > 1e-6 and np.linalg.norm(seg[:2]) < 1e-6 else feed
        n = max(1, int(np.ceil(L / (speed * ctrl_dt))))
        for k in range(1, n + 1):
            pts.append(a + seg * (k / n))
    return np.array(pts)


class ArmIK:
    """6-DOF damped-least-squares IK for the Lite 6's `tcp` site, tool held
    pointing straight down. Runs on its own MjData so it never disturbs a live
    simulation state passed in from the caller."""

    def __init__(self, model, tcp_site="tcp", joints=None, damp=1e-4):
        self.model = model
        self.ikd = mujoco.MjData(model)
        self.tcp = model.site(tcp_site).id
        joints = joints or [f"joint{i}" for i in range(1, 7)]
        jids = [model.joint(j).id for j in joints]
        self.qadr = np.array([model.jnt_qposadr[j] for j in jids])
        self.dadr = np.array([model.jnt_dofadr[j] for j in jids])
        self.qlo = np.array([model.jnt_range[j, 0] for j in jids])
        self.qhi = np.array([model.jnt_range[j, 1] for j in jids])
        self.damp = damp
        self.jacp = np.zeros((3, model.nv))
        self.jacr = np.zeros((3, model.nv))
        R_target = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
        self.quat_target = np.zeros(4)
        mujoco.mju_mat2Quat(self.quat_target, R_target.ravel())

    def pose_error(self, q, target_pos):
        """6-vector [pos; rot] error of the tcp site at joint config q."""
        self.ikd.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, self.ikd)
        mujoco.mj_comPos(self.model, self.ikd)
        pos_err = target_pos - self.ikd.site_xpos[self.tcp]
        q_cur = np.zeros(4)
        mujoco.mju_mat2Quat(q_cur, self.ikd.site_xmat[self.tcp])
        q_err = np.zeros(4)
        mujoco.mju_mulQuat(q_err, self.quat_target,
                            np.array([q_cur[0], -q_cur[1], -q_cur[2], -q_cur[3]]))
        if q_err[0] < 0:                       # take the short way round
            q_err = -q_err
        rot_err = np.zeros(3)
        mujoco.mju_quat2Vel(rot_err, q_err, 1.0)
        return np.hstack([pos_err, rot_err])

    def solve(self, target_pos, q0, iters=80, tol=1e-4):
        """Warm-started DLS IK. Pure kinematic (no dynamics)."""
        q = np.array(q0, dtype=float)
        for _ in range(iters):
            err = self.pose_error(q, target_pos)
            if np.linalg.norm(err[:3]) < tol and np.linalg.norm(err[3:]) < 1e-3:
                break
            mujoco.mj_jacSite(self.model, self.ikd, self.jacp, self.jacr, self.tcp)
            J = np.vstack([self.jacp[:, self.dadr], self.jacr[:, self.dadr]])
            dq = J.T @ np.linalg.solve(J @ J.T + self.damp * np.eye(6), err)
            q = np.clip(q + dq, self.qlo, self.qhi)
        return q


def settle_to_start(model, data, ik, target0, qadr, dadr, settle_steps=300):
    """Drive the arm to `target0` with IK, zero velocity, then step physics to settle."""
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    q_des = ik.solve(target0, data.qpos[qadr].copy(), iters=400)
    data.qpos[qadr] = q_des
    data.qvel[dadr] = 0.0
    data.ctrl[:] = q_des
    mujoco.mj_forward(model, data)
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)
    return q_des
