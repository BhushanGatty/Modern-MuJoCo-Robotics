import mujoco
import mujoco.viewer
import time

# 1. THE XML (Rearranged: Red - Blue - Green)
xml = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light name="top" pos="0 0 10"/>
    
    <geom name="floor" type="plane" size="10 10 .1" rgba=".8 .9 .8 1" solref="0.002 0"/>
    
    <body name="ball_mud" pos="0 0 2">
      <joint type="free"/>
      <geom type="sphere" size=".2" rgba="1 0 0 1" solref="0.02 1.0"/>
    </body>
    
    <body name="ball_super" pos="-1 0 2">
      <joint type="free"/>
      <geom type="sphere" size=".2" rgba="0 0 1 1" solref="0.002 0"/>
    </body>

    <body name="ball_sport" pos="1 0 2">
      <joint type="free"/>
      <geom type="sphere" size=".2" rgba="0 1 0 1" solref="0.02 0.3"/>
    </body>

  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("Blue ball should now bounce almost forever.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)
        
        # Reset every 8 seconds
        if time.time() - start_time > 8.0:
            start_time = time.time()
            # Reset Positions (Z=2)
            data.qpos[model.jnt_qposadr[0]+2] = 2 
            data.qpos[model.jnt_qposadr[1]+2] = 2 
            data.qpos[model.jnt_qposadr[2]+2] = 2 
            # Reset Velocity
            data.qvel[:] = 0