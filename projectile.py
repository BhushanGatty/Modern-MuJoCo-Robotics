import mujoco
import mujoco.viewer
import time

# 1. THE XML (The Physical World)
# We define a "red_ball" with a "free joint" so it can move anywhere.
xml = """
<mujoco>
<option gravity="0 0 -9.81"/>
<worldbody>
<light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
<geom type="plane" size="1 1 0.1" rgba="0 0.9 0 1"/>
<body pos="0 0 1">
<joint type="free"/>
<geom type="box" size=".1 .2 .3" rgba="0.9 0 0 1"/>
</body>
<body pos="0.1 0 1.5" euler="0 90 0">
<joint type="free"/>
<geom type="box" size=".1 .2 .3" rgba="0.1 0.8 0.9 1"/>
</body>
<body pos="0.2 0 1.8">
<joint type="free"/>
<geom type="sphere" size=".1" rgba="0.7 0.7 0.7 1"/>
</body>
</worldbody>
</mujoco>
"""

# 2. LOAD THE MODEL
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 3. SET INITIAL CONDITIONS (The "Throw")
# qvel = Joint Velocity. 
# Index 0 = X axis (forward), Index 2 = Z axis (up)
data.qvel[0] = 5   # Throw forward at 5 m/s
data.qvel[2] = 10  # Throw up at 10 m/s

# 4. RUN THE SIMULATION (Passive Mode)
# We use "launch_passive" so we can see the physics happen
print("Launching Projectile... Check the window!")
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    # Loop while the viewer is open
    while viewer.is_running():
        
        # Step the physics engine forward
        mujoco.mj_step(model, data)

        # Update the screen
        viewer.sync()

        # Slow down so we can see it (real-time roughly)
        time.sleep(0.002)