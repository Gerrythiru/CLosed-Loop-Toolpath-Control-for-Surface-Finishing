"""Verify the emulated OptiTrack PrimeX 22 mocap rig (3 static, external cameras).

1. Checks every camera clears the two keep-out constraints:
     - table-height keep-out: camera must be above table_top + 0.70 m
     - arm-sweep keep-out: camera must be outside a sphere at the Lite 6 shoulder
       (radius = kinematic max reach + tool + margin)
2. Renders one image from each of the 3 cameras at the home pose (checking the
   plate, its 4 corner markers, the table, and the arm are framed).
3. Renders one external overview shot showing the rig (truss + cameras) in context.

Run:  python check_mocap_rig.py
"""
import numpy as np
import mujoco
import PIL.Image

TABLE_TOP_Z = 0.75
KEEPOUT_HEIGHT = 0.70              # no mounting hardware within this of the table top
ARM_SPHERE_RADIUS = 0.60           # kinematic max reach (~0.55 m) + tool + margin

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.mj_forward(model, data)

shoulder = data.xpos[model.body("link1").id].copy()
print(f"arm shoulder (keep-out sphere center) = {np.round(shoulder, 3)}, "
      f"radius = {ARM_SPHERE_RADIUS} m")
print(f"table-height keep-out: z <= {TABLE_TOP_Z + KEEPOUT_HEIGHT:.2f} m is off-limits")
print("-" * 72)

ok = True
for i in range(model.ncam):
    name = model.camera(i).name
    pos = model.cam_pos[i]
    height_clear = pos[2] - (TABLE_TOP_Z + KEEPOUT_HEIGHT)
    sphere_clear = np.linalg.norm(pos - shoulder) - ARM_SPHERE_RADIUS
    good = height_clear > 0 and sphere_clear > 0
    ok &= good
    print(f"{name:16s} pos={np.round(pos,3)}  height-clear={height_clear:+.2f} m  "
          f"arm-sphere-clear={sphere_clear:+.2f} m  {'OK' if good else 'VIOLATION'}")

print("-" * 72)
print("RESULT:", "ALL CAMERAS CLEAR KEEP-OUT ZONES" if ok else "KEEP-OUT VIOLATION")

# ------------------------------------------------------------------ render the views
ren = mujoco.Renderer(model, 544, 1024)   # ~1.88:1, matching the PrimeX 22 sensor aspect

for i in range(model.ncam):
    name = model.camera(i).name
    ren.update_scene(data, camera=name)
    PIL.Image.fromarray(ren.render()).save(f"mocap_view_{name}.png")
    print(f"wrote mocap_view_{name}.png")

# external overview shot (not one of the 3 mocap cameras)
ren2 = mujoco.Renderer(model, 720, 1280)
cam = mujoco.MjvCamera()
cam.lookat[:] = [-0.15, 0, 1.0]
cam.distance, cam.azimuth, cam.elevation = 3.4, 205, -22
ren2.update_scene(data, cam)
PIL.Image.fromarray(ren2.render()).save("mocap_rig_overview.png")
print("wrote mocap_rig_overview.png")
