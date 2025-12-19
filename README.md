# Modern MuJoCo Robotics (2025)

A collection of robotics algorithms (Kinematics, Dynamics, Control) based on Pranav Bhounsule's bootcamp, fully refactored and modernized for the new DeepMind `mujoco` Python bindings.

## The Project
The original course uses the deprecated `mujoco-py` wrapper which no longer works on modern Linux systems. This repository contains the **modernized translations** of those algorithms using the native DeepMind API.

## Tech Stack
* **Language:** Python 3.10+
* **Physics Engine:** DeepMind MuJoCo (2.x/3.x)
* **IDE:** VS Code (Linux)

## Examples Included
* **Projectile Motion:** Basic physics and  simulation.
* ![Adobe Express - Adobe Express - video_20251216_135918_edit](https://github.com/user-attachments/assets/e26b95e8-b093-4ea6-a970-5d8fb5d92760)

* **2-Link Manipulator:** Double pendulum dynamics and chaotic motion.
![Adobe Express - Adobe Express - video_20251216_133054_edit](https://github.com/user-attachments/assets/b545936d-23f1-42bd-a1cc-c2966c588711)

## 3. 3D Inverse Kinematics (Reaching)
* **File:** `3d_reach.py`
* **Description:** Upgraded the system to a 3-DOF (Degree of Freedom) robotic arm capable of moving in 3D space (X, Y, Z). 
* **Math Used:** Implements Jacobian Inverse Kinematics with Damping (`mu_jacSite` and `numpy.linalg.pinv`) to solve for joint angles based on a 3D target position.
 ![3d_arm_GIFS](https://github.com/user-attachments/assets/b983b174-28e8-4e65-9398-6f0f2b63b901)
