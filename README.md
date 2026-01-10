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

## 4. Mobile Robotics (Differential Drive)
* **File:** `car.py`
* **Description:** A simulation of a 3-wheeled differential drive robot (like a Roomba or Pioneer robot).
* **Physics Features:**
    * Implements a **Skid-Steering** kinematic model.
    * Uses a **Free Joint** (6DOF) to allow navigation anywhere in the environment.
    * Features a friction-less caster wheel for stability.
* **Control Logic:** A state-machine loop that executes a "Figure 8" driving pattern (Forward -> Left Turn -> Right Turn -> Spin).

### Demo
![Untitled design](https://github.com/user-attachments/assets/d1f2c805-ebf1-4bbf-9fb2-c5fc7f6baef4)

## 5. Computational Physics (Drag Force)
* **File:** `7_projectile_drag.py`
* **Description:** A physics experiment comparing ideal motion vs. real-world air resistance.
* **The Physics:**
    * **Red Ball:** Vacuum physics (Ideal Parabola).
    * **Blue Ball:** Applies a custom drag force $F = -c \cdot v$ injected directly into the simulation loop.
* **Implementation:** Uses `data.qfrc_applied` to apply custom forces to specific degrees of freedom at every simulation step.

## 6. Contact Physics (Restitution)
* **File:** `8_bouncing.py`
* **Description:** Demonstrates how to control the Coefficient of Restitution (Bounciness) using MuJoCo's `solref` parameters.
* **The Setup (Left to Right):**
    * **Red:** High Damping (Inelastic / Mud).
    * **Blue:** Zero Damping (Elastic / Superball).
    * **Green:** Medium Damping (Basketball).
* **Key Concept:** Shows how we can tune the simulation to model different material properties, from energy-absorbing clay to high-energy rubber.

### Demo
![Untitled design (1)](https://github.com/user-attachments/assets/62afddeb-e34a-461f-9c62-13a58f434634)
