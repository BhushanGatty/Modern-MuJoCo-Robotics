import mujoco
import mujoco.viewer
import time

# 1. THE XML (Double Pendulum)
xml = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <geom name="floor" type="plane" size="2 2 .1" rgba=".8 .9 .8 1"/>
    
    <body name="link1" pos="0 0 2">
      <joint name="pin1" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" size=".05" fromto="0 0 0 0 0 -1" rgba="1 0 0 1"/>
      
      <body name="link2" pos="0 0 -1">
        <joint name="pin2" type="hinge" axis="0 1 0" damping="0.1"/>
        <geom type="capsule" size=".05" fromto="0 0 0 0 0 -1" rgba="0 1 0 1"/>
        <site name="tip" pos="0 0 -1" size=".1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# --- THE FIX: STARTING CONDITIONS ---
print("Applying initial push...")
data.qpos[1] = 1.5  # Start the Green link bent at 90 degrees
data.qvel[0] = 10   # Spin the Red link hard!

# Run Simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    while viewer.is_running():
        
        # Step physics
        mujoco.mj_step(model, data)
        
        # Update screen
        viewer.sync()
        
        # Slow down slightly so we can see the chaos
        time.sleep(0.002)