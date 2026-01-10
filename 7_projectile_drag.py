import mujoco
import mujoco.viewer
import time
import numpy as np

# 1. THE XML (Two Spheres)
xml = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light name="top" pos="0 0 10"/>
    <geom name="floor" type="plane" size="10 10 .1" rgba=".8 .9 .8 1"/>
    
    <body name="ball_no_drag" pos="0 -1 1">
      <joint type="free"/>
      <geom type="sphere" size=".2" rgba="1 0 0 1"/> </body>
    
    <body name="ball_drag" pos="0 1 1">
      <joint type="free"/>
      <geom type="sphere" size=".2" rgba="0 0 1 1"/> </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 2. SET INITIAL CONDITIONS
# Launch both balls forward (X-axis) and Up (Z-axis)
# qvel structure for free joint: [vx, vy, vz, wx, wy, wz]
ball1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball_no_drag")
ball2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball_drag")

# Get the address in the velocity array for these bodies
dof1 = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "ball_no_drag")]
dof2 = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "ball_drag")]

# Launch! (Velocity = 10 m/s forward, 10 m/s up)
data.qvel[dof1:dof1+3] = [10, 0, 10]
data.qvel[dof2:dof2+3] = [10, 0, 10]

print("Red = Vacuum (Parabola)")
print("Blue = Drag (Short fall)")

# Drag Coefficient (How "thick" the air is)
c = 10 

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        
        # --- PHYSICS INJECTION ---
        
        # 1. Get Velocity of Blue Ball
        # We access the velocity from the 'qvel' array
        vx = data.qvel[dof2]
        vy = data.qvel[dof2+1]
        vz = data.qvel[dof2+2]
        
        # 2. Calculate Drag Force
        # Equation: F = -c * velocity
        fx = -c * vx
        fy = -c * vy
        fz = -c * vz
        
        # 3. Apply Force
        # qfrc_applied is a bucket where we can dump custom forces
        data.qfrc_applied[dof2] = fx
        data.qfrc_applied[dof2+1] = fy
        data.qfrc_applied[dof2+2] = fz
        
        # -------------------------

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)
        
        # Reset if they fall through floor
        if data.body("ball_no_drag").xpos[2] < -1:
             data.qpos[model.jnt_qposadr[dof1]:model.jnt_qposadr[dof1]+7] = [0, -1, 1, 1, 0, 0, 0] # Reset Pos
             data.qpos[model.jnt_qposadr[dof2]:model.jnt_qposadr[dof2]+7] = [0, 1, 1, 1, 0, 0, 0]
             data.qvel[:] = 0
             data.qvel[dof1:dof1+3] = [10, 0, 10]
             data.qvel[dof2:dof2+3] = [10, 0, 10]