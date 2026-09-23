"""Verify the Lite 6 EE (tcp site) can reach every point on the work plate.

Loads scene.xml and runs multi-seed damped-least-squares position IK from the
`tcp` site to a grid of targets on the plate's top surface. A target counts as
reachable if any seed converges to within TOL. Reports per-target error and a
pass/fail for the whole plate.
"""
import numpy as np
import mujoco

TOL = 1e-3      # 1 mm
SEEDS = 40      # random restarts per target
GRID = 9        # GRID x GRID targets across the plate

rng = np.random.default_rng(0)
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
tcp = model.site("tcp").id

# plate extent straight from the model
pc = model.body("plate").pos.copy()
hx, hy, hz = model.geom("plate_geom").size
top_z = pc[2] + hz
xs = np.linspace(pc[0] - hx, pc[0] + hx, GRID)
ys = np.linspace(pc[1] - hy, pc[1] + hy, GRID)
targets = [np.array([x, y, top_z]) for x in xs for y in ys]

jids = [model.joint(f"joint{i}").id for i in range(1, 7)]
qadr = [model.jnt_qposadr[j] for j in jids]
lo = np.array([model.jnt_range[j, 0] for j in jids])
hi = np.array([model.jnt_range[j, 1] for j in jids])
jacp = np.zeros((3, model.nv))

def solve(target, q0, iters=4000, damp=1e-5):
    q = q0.copy()
    for _ in range(iters):
        data.qpos[qadr] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        err = target - data.site_xpos[tcp]
        if np.linalg.norm(err) < TOL:
            break
        mujoco.mj_jacSite(model, data, jacp, None, tcp)
        J = jacp[:, qadr]
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + damp * np.eye(3), err), lo, hi)
    return np.linalg.norm(err)

def best_ik(target):
    b = np.inf
    for _ in range(SEEDS):
        b = min(b, solve(target, rng.uniform(lo, hi)))
        if b < TOL:
            break
    return b

mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.mj_forward(model, data)
print(f"plate: {2*hx:.2f} m x {2*hy:.2f} m, top surface z = {top_z:.3f} m")
print(f"       x in [{pc[0]-hx:+.3f}, {pc[0]+hx:+.3f}], y in [{pc[1]-hy:+.3f}, {pc[1]+hy:+.3f}]")
print(f"arm base at {model.body('link_base').pos},  shoulder z = {data.xpos[model.body('link1').id][2]:.3f} m")
print("-" * 60)

errs = np.array([best_ik(t) for t in targets])
n_bad = int((errs >= TOL).sum())
for i in np.argsort(errs)[::-1][:5]:
    t = targets[i]
    print(f"  ({t[0]:+.3f}, {t[1]:+.3f})  err = {errs[i]*1000:.3f} mm")
print("-" * 60)
print(f"targets: {len(targets)}   unreachable (> {TOL*1000:.0f} mm): {n_bad}   "
      f"worst: {errs.max()*1000:.3f} mm")
print("RESULT:", "PLATE FULLY REACHABLE" if n_bad == 0 else "PLATE NOT FULLY REACHABLE")
