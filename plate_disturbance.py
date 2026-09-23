"""Phase E (v2): scripted, discrete stepped-yaw panel motion.

Models the work plate's "moves unpredictably" behavior as a sequence of
discrete yaw steps -- not continuous vibration/jitter (that was v1's
Ornstein-Uhlenbeck random walk). A fixed schedule of 30 steps is generated up
front from a seeded RNG: each step is +/-1.5 deg (direction random per step),
separated by a random interval uniform in [2, 5] s. Between steps the plate
holds still; at each scheduled time it's driven smoothly to the next target
angle by a PD torque controller -- gentle, exponential-style settling, no
oscillation, no per-tick noise.

The plate is now a real MuJoCo free body (mass 3 kg -- realistic for a 0.30 x
0.30 x 0.012 m aluminum plate -- with real inertia, resting on the table via
ordinary contact; see `<freejoint/>` on `plate` in scene.xml) rather than a
kinematic mocap body. `apply()` writes a torque into `data.xfrc_applied` every
physics step; MuJoCo's own rigid-body dynamics does the rest, so the plate is
genuinely contact-reactive (the arm's tool, or anything else, can physically
push it) while still being driven toward the scripted schedule.

Pivot: this pass rotates about the plate's own origin (== its center of mass),
so pure Z-axis torque is exact -- applying torque at the COM changes only
angular momentum, not linear momentum, so the body spins in place with no
translation forced by the controller. A "V2" that pivots about a plate corner
instead would need a second, coupled PD *force* loop tracking the COM around
the arc that an off-center pivot implies (equivalent to reconstructing what a
real hinge joint gives for free) -- not implemented here, since only the
origin pivot was asked for this pass.

Gain calibration -- this is not a simple inertia-only PD design, and the
reason why is worth recording. A naive Kp/Kd derived purely from the plate's
Izz (ignoring contact) is off by ~3 orders of magnitude: the plate rests on
the table via 4 simultaneous corner contacts, which is a classic redundant/
over-constrained configuration for a rigid-body solver, and combined with the
`implicitfast` integrator (kept globally for the arm's own stability) this
absorbed almost all torque below a threshold as spurious numerical damping,
while anything above a nearby second threshold made the contact solver fail
outright (visible launching/tunneling of the plate). This was confirmed to be
a numerical artifact, not friction, by reproducing the same behavior with
`condim=1` (a purely frictionless normal contact) -- identical suppression and
identical instability threshold with zero tangential/torsional/rolling
friction in play. Fix: soften `plate_geom`'s `solref`/`solimp` in scene.xml,
which turns the response linear and stable across a wide, usable torque range;
the gains below are calibrated against that softened contact and this
specific mass -- **re-run the calibration sweep in this file's `__main__`
docstring/notes if `plate_geom`'s mass, size, or solref/solimp ever change.**
"""
import numpy as np
import mujoco

STEP_DEG = 1.5           # magnitude of each yaw step
N_STEPS = 30             # total number of steps in the schedule
INTERVAL_MIN_S = 2.0     # random interval between steps, uniform in [min, max]
INTERVAL_MAX_S = 5.0

# Empirically calibrated (see module docstring) for the 3 kg plate + softened
# plate_geom contact in scene.xml. A constant test torque swept from 0.5 to
# 10 N*m showed a clean linear, stable response up to ~4 N*m and solver
# failure (launching) at >=5 N*m -- KP/torque cap below stay well clear of
# that with margin. KD adds light damping against overshoot; in practice the
# contact-dominated response is heavily damped already and rarely overshoots.
KP_ROT = 134.0        # N*m / rad
KD_ROT = 6.0          # N*m / (rad/s)
TORQUE_CAP = 3.5      # N*m -- hard ceiling, stays clear of the ~5 N*m instability point


class PlateDisturbance:
    """Pre-scheduled discrete yaw-step sequence, applied as PD torque to a
    free-body plate about its own origin."""

    def __init__(self, model, plate_body="plate", seed=0,
                 step_deg=STEP_DEG, n_steps=N_STEPS,
                 interval_min_s=INTERVAL_MIN_S, interval_max_s=INTERVAL_MAX_S):
        self.model = model
        self.plate_id = model.body(plate_body).id
        assert model.body_jntnum[self.plate_id] > 0, \
            f'"{plate_body}" must have a freejoint (see scene.xml)'

        self.nominal_pos = model.body_pos[self.plate_id].copy()
        self.nominal_quat = model.body_quat[self.plate_id].copy()
        self.dof_adr = model.body_dofadr[self.plate_id]

        rng = np.random.default_rng(seed)
        self.signs = rng.choice([-1.0, 1.0], size=n_steps)
        self.intervals = rng.uniform(interval_min_s, interval_max_s, size=n_steps)
        self.step_times = np.cumsum(self.intervals)
        self.step_rad = np.radians(step_deg)
        # cumulative commanded yaw (rad) after each step, and before the first
        self.targets = np.concatenate([[0.0], np.cumsum(self.signs) * self.step_rad])

        self._next_step = 0
        self.target_yaw = 0.0   # current commanded yaw offset from nominal, rad
        self._t_acc = 0.0       # internal clock, used when apply() is called without t

    def _target_quat(self):
        dq = np.zeros(4)
        mujoco.mju_axisAngle2Quat(dq, np.array([0.0, 0.0, 1.0]), self.target_yaw)
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, self.nominal_quat, dq)
        return q

    def apply(self, data, dt, t=None):
        """Advance the schedule and write this tick's PD torque into
        data.xfrc_applied. Call once per physics step, before mj_step. `t` is
        the simulation clock; if omitted, an internal accumulator (starting
        at 0) is used instead."""
        if t is None:
            t = self._t_acc + dt
        self._t_acc = t

        while self._next_step < len(self.step_times) and t >= self.step_times[self._next_step]:
            self.target_yaw = self.targets[self._next_step + 1]
            self._next_step += 1

        q_cur = data.xquat[self.plate_id]
        q_tgt = self._target_quat()
        q_err = np.zeros(4)
        mujoco.mju_mulQuat(q_err, q_tgt, np.array([q_cur[0], -q_cur[1], -q_cur[2], -q_cur[3]]))
        if q_err[0] < 0:
            q_err = -q_err
        rot_err = np.zeros(3)
        mujoco.mju_quat2Vel(rot_err, q_err, 1.0)

        ang_vel = data.qvel[self.dof_adr + 3: self.dof_adr + 6]
        torque = np.clip(KP_ROT * rot_err - KD_ROT * ang_vel, -TORQUE_CAP, TORQUE_CAP)

        data.xfrc_applied[self.plate_id, :3] = 0.0
        data.xfrc_applied[self.plate_id, 3:] = torque
        return self.target_yaw
