import mujoco
import mujoco.viewer
import time
import numpy as np

# 1. THE XML (Fixed Stability)
xml = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light name="top" pos="0 0 10"/>
    <geom name="floor" type="plane" size="10 10 .1" rgba=".8 .9 .8 1"/>
    
    <body name="car" pos="0 0 0.2"> <joint type="free"/>
      
      <geom type="box" size=".2 .1 .05" rgba=".9 .5 .1 1"/>
      <geom type="box" size=".05 .05 .05" pos=".1 0 .05" rgba="0 0 1 1"/> <body name="wheel_left" pos=".1 .15 0">
        <joint name="joint_left" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size=".1 .05" zaxis="0 1 0" rgba=".2 .2 .2 1"/>
      </body>
      
      <body name="wheel_right" pos=".1 -.15 0">
        <joint name="joint_right" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size=".1 .05" zaxis="0 1 0" rgba=".2 .2 .2 1"/>
      </body>
      
      <geom name="caster" type="sphere" size=".05" pos="-.15 0 -.05" rgba=".5 .5 .5 1" friction="0 0 0"/>
    </body>
  </worldbody>

  <actuator>
    <velocity name="motor_left"  joint="joint_left"  kv="10"/>
    <velocity name="motor_right" joint="joint_right" kv="10"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("Driving School Open! (Stable Version)")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        now = time.time() - start_time
        
        # --- DRIVING LOGIC ---
        if now < 2:
            left, right = 5, 5   # Forward
        elif now < 4:
            left, right = 2, 8   # Turn Left
        elif now < 6:
            left, right = 8, 2   # Turn Right
        else:
            left, right = -5, 5  # Spin
            
        if now > 8: start_time = time.time()

        data.ctrl[0] = left
        data.ctrl[1] = right
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Camera follows car
        # Update screen
        
        
        # Follow the car's POSITION, but let the user control ZOOM/ROTATION
        viewer.cam.lookat[:] = data.body("car").xpos
        # viewer.cam.distance = 5  <-- DELETE THIS LINE
        
        time.sleep(0.002)