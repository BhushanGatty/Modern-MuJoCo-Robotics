import mujoco
import mujoco.viewer
import time
import numpy as np

# 1. THE XML (3D ROBOT)
xml = """
<mujoco>
  <option gravity="0 0 0"/> <worldbody>
    <light name="top" pos="0 0 1.5"/>
    <geom name="floor" type="plane" size="2 2 .1" rgba=".8 .9 .8 1"/>
    
    <body name="base" pos="0 0 0">
      <joint name="waist" type="hinge" axis="0 0 1" damping="0.5"/>
      <geom type="cylinder" size=".1 .05" rgba=".5 .5 .5 1"/>
      
      <body name="upper_arm" pos="0 0 .1">
        <joint name="shoulder" type="hinge" axis="0 1 0" damping="0.5"/>
        <geom type="capsule" size=".05" fromto="0 0 0 0 0 .5" rgba="1 0 0 1"/>
        
        <body name="forearm" pos="0 0 .5">
          <joint name="elbow" type="hinge" axis="0 1 0" damping="0.5"/>
          <geom type="capsule" size=".04" fromto="0 0 0 0 0 .4" rgba="0 1 0 1"/>
          
          <site name="hand" pos="0 0 .4" size=".05" rgba="0 0 1 1"/>
        </body>
      </body>
    </body>

    <body name="target" pos="0.3 0 0.5" mocap="true">
      <geom type="sphere" size=".05" rgba="0 1 0 0.5"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 2. RUN SIMULATION
print("Running 3D Reach... Use Ctrl + Right Click to lift the target!")
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        
        # --- INVERSE KINEMATICS (Standard) ---
        hand_pos = data.site("hand").xpos
        target_pos = data.body("target").xpos
        error = target_pos - hand_pos
        
        # Calculate Jacobian (Now 3x3 because we have 3 joints!)
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, None, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hand"))
        
        # Solve
        qvel_target = np.linalg.pinv(jacp) @ error
        data.qvel[:] = qvel_target * 5
        # -------------------------------------

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)