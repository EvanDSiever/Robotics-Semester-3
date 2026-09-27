# 4-DOF Robotic Arm Manipulator & Kinematics Engine
**Robotics Semester 3 Project: Vectors, Matrices, and End-Effector Kinematics**

![Robotic Arm Kinematics](https://img.shields.io/badge/PlatformIO-Arduino%20Uno-blue?logo=platformio)
![Kinematics](https://img.shields.io/badge/Math-Vectors%20%26%20Matrices-brightgreen)
![Status](https://img.shields.io/badge/Build-Passing%20%26%20Flashed-success)

---

## 1. Project Overview

This project implements a **4-Degree-of-Freedom (4-DOF) Robotic Arm Manipulator** powered by 4 Micro SG90 servomotors controlled by an **Arduino Uno (ATmega328P)**. 

The firmware bridges fundamental linear algebra and classical mechanics:
- **Vectors**: Link translations, position vectors $\vec{p}$, and Euclidean distances.
- **Transformation Matrices**: $4 \times 4$ Homogeneous matrices $T$ combining 3D rotation and translation.
- **Forward Kinematics (FK)**: Determining the exact spatial 3D position vector of the End-Effector (clamp) from given joint angles $(\theta_1, \theta_2, \theta_3)$.
- **Inverse Kinematics (IK)**: Determining the required joint angles to reach any target $(X, Y, Z)$ position vector in space using geometric trigonometry (Law of Cosines).
- **Smooth Trajectory Interpolation**: Eliminates harsh jerks and mechanical oscillation by linearly interpolating multi-joint movements.
- **Interactive Serial Monitor Interface**: Allows direct command execution, real-time matrix inspection, and an automated Pick-and-Place demonstration sequence.

---

## 2. Hardware Architecture & Critical Wiring Guide

> [!CAUTION]
> ### CRITICAL ELECTRICAL SAFETY & WIRING FIX
> In your original setup, you noted: *"Their 5 V pins are not connected to anything (which I assume is what I should do)..."*
> 
> **Servos CANNOT operate without 5V connected to their power pins!**
> 
> - **SG90 Pinout**:
>   - **Brown / Black Wire**: Ground (`GND`)
>   - **Red Wire**: Power (`+5V`)
>   - **Orange / Yellow Wire**: PWM Signal Pin
> 
> - **Why you MUST NOT power the servos directly from the Arduino's 5V pin**:
>   4 SG90 servos moving simultaneously can draw peak currents of **1.5A to 2.0A+**. The Arduino Uno onboard 5V regulator can only output ~400mA. If you power 4 servos from the Arduino's 5V pin, the board will suffer severe voltage drops (brownouts), constantly restart, or overheat the regulator.
> 
> - **The Correct Power Supply Architecture**:
>   1. Connect the **+5V terminal of your external 5V 2A power supply** to the **Red (+) power rail** of your breadboard.
>   2. Connect all **4 Red wires of the SG90 servos** directly to that **Red (+) breadboard rail**.
>   3. Connect the **GND (-) terminal of your external 5V 2A power supply** to the **Blue/Black (-) ground rail** of the breadboard.
>   4. Connect all **4 Brown/Black wires of the SG90 servos** to that **ground rail**.
>   5. **MANDATORY COMMON GROUND**: Run a jumper wire from that **Blue/Black (-) breadboard ground rail** to any **`GND` pin on the Arduino Uno**. Without this shared ground, the PWM signals have no reference voltage and the servos will twitch erratically or fail to respond.
>   6. The Arduino Uno itself is powered safely through the USB cable connected to your laptop.

### Pin Assignment Summary

| Joint / Actuator | Mechanical Function | Arduino Uno PWM Pin | Range of Motion | Home Position |
| :--- | :--- | :--- | :--- | :--- |
| **Base Joint (Yaw)** | Rotates the entire arm structure horizontally | **Digital Pin 9** | $0^\circ \text{ to } 180^\circ$ | $90^\circ$ (Forward, $+X$) |
| **Shoulder Joint** | Pitch elevation of the upper arm link | **Digital Pin 3** | $15^\circ \text{ to } 165^\circ$ | $90^\circ$ |
| **Elbow Joint** | Pitch elevation of the forearm link | **Digital Pin 11** | $15^\circ \text{ to } 165^\circ$ | $90^\circ$ |
| **Clamp / Gripper** | End-Effector grasping mechanism | **Digital Pin 5** | $30^\circ \text{ (Closed)} - 100^\circ \text{ (Open)}$ | $100^\circ$ (Open) |

---

## 3. Robotics Kinematics Theory

### 3.1 What is an End-Effector?
In robotics, the **End-Effector** (often abbreviated as **EE**) is the final active device at the end of a robotic manipulator arm designed to interact with the physical environment. In your arm, the **gripper clamp** is the end-effector. The point in space whose coordinates we track and position is called the **Tool Center Point (TCP)**.

```
       [Elbow Joint] (Pin 11)
          O===================\
         //       Link 3       \
        // (Forearm: 9 cm)      \
Link 2 //                        [Wrist / Clamp] (Pin 5)
(Upper Arm: 12 cm)               {===} <-- End-Effector (TCP)
      //
     O [Shoulder Joint] (Pin 3)
     |
     | Link 1 (Base Height: 4.5 cm)
    === [Base Yaw Joint] (Pin 9)
  ------- [Mounting Ground Surface / Origin (0,0,0)]
```

---

### 3.2 Vectors in Robotics
A vector in 3-dimensional Euclidean space is represented as:

$$\vec{p} = \begin{bmatrix} x \\ y \\ z \end{bmatrix} \in \mathbb{R}^3$$

1. **Link Vectors**:
   Each arm segment forms a directed vector:
   - $\vec{r}_1$: Vector from ground origin to shoulder pivot: $\begin{bmatrix} 0 \\ 0 \\ L_1 \end{bmatrix}$
   - $\vec{r}_2$: Vector from shoulder to elbow: $\begin{bmatrix} L_2 \cos\phi_2 \cos\phi_1 \\ L_2 \cos\phi_2 \sin\phi_1 \\ L_2 \sin\phi_2 \end{bmatrix}$
   - $\vec{r}_3$: Vector from elbow to clamp end-effector: $\begin{bmatrix} L_3 \cos\phi_3 \cos\phi_1 \\ L_3 \cos\phi_3 \sin\phi_1 \\ L_3 \sin\phi_3 \end{bmatrix}$

2. **Vector Addition**:
   By vector addition, the total position vector $\vec{p}$ of the End-Effector relative to the base origin is:

   $$\vec{p}_{\text{EE}} = \vec{r}_1 + \vec{r}_2 + \vec{r}_3 = \begin{bmatrix} x \\ y \\ z \end{bmatrix}$$

3. **Euclidean Vector Norm (Distance)**:
   The straight-line distance from the base origin to the clamp is given by the Euclidean norm:

   $$\|\vec{p}\| = \sqrt{x^2 + y^2 + z^2}$$

---

### 3.3 Homogeneous Transformation Matrices ($4 \times 4$)

In robotics, moving between coordinate frames requires both **Rotation** ($3 \times 3$ matrix $R$) and **Translation** ($3 \times 1$ vector $\vec{d}$). To combine both into a single linear matrix operation, we use **Homogeneous Coordinates**:

$$T = \begin{bmatrix} R_{3\times3} & \vec{d}_{3\times1} \\ \mathbf{0}_{1\times3} & 1 \end{bmatrix} = \begin{bmatrix} 
r_{11} & r_{12} & r_{13} & d_x \\
r_{21} & r_{22} & r_{23} & d_y \\
r_{31} & r_{32} & r_{33} & d_z \\
0 & 0 & 0 & 1
\end{bmatrix}$$

For our manipulator:
1. **Base Frame 0 to Shoulder Frame 1**:
   Rotates by base yaw angle $\phi_1 = (\theta_1 - 90^\circ)$ around $Z$-axis and translates upwards by $L_1$:

   $$T_0^1 = \begin{bmatrix}
   \cos\phi_1 & -\sin\phi_1 & 0 & 0 \\
   \sin\phi_1 & \cos\phi_1 & 0 & 0 \\
   0 & 0 & 1 & L_1 \\
   0 & 0 & 0 & 1
   \end{bmatrix}$$

2. **Sequential Multi-Joint Transformation**:
   By chaining the transformation matrices through matrix multiplication:

   $$T_0^{\text{EE}} = T_0^1 \times T_1^2 \times T_2^3$$

   The 4th column of $T_0^{\text{EE}}$ gives the exact $(X, Y, Z)$ position vector of the End-Effector!

---

### 3.4 Forward Kinematics (FK)
Given the servo angles $(\theta_1, \theta_2, \theta_3)$:
1. **Base Yaw Angle**:
   $$\phi_1 = (\theta_1 - 90^\circ) \cdot \frac{\pi}{180}$$
2. **Shoulder Elevation Angle**:
   $$\phi_2 = \theta_2 \cdot \frac{\pi}{180}$$
3. **Forearm Absolute Pitch Angle**:
   $$\phi_3 = \phi_2 + (\theta_3 - 180^\circ) \cdot \frac{\pi}{180}$$
4. **Planar Radius $r$ and Height $z$**:
   $$r = L_2 \cos\phi_2 + L_3 \cos\phi_3$$
   $$z = L_1 + L_2 \sin\phi_2 + L_3 \sin\phi_3$$
5. **3D Cartesian Position**:
   $$x = r \cos\phi_1$$
   $$y = r \sin\phi_1$$
   $$z = z$$

---

### 3.5 Inverse Kinematics (IK)
Given a desired target position vector $\vec{p} = [x, y, z]^T$:
1. **Solve Base Angle ($\theta_1$)**:
   $$\phi_1 = \text{atan2}(y, x)$$
   $$\theta_1 = 90^\circ + \phi_1 \cdot \frac{180}{\pi}$$

2. **Project into Arm Sagittal Plane**:
   $$r = \sqrt{x^2 + y^2}, \quad z' = z - L_1$$
   $$D = \sqrt{r^2 + (z')^2} \quad \text{(Distance from shoulder pivot to target)}$$

3. **Workspace Boundary Verification**:
   The arm can only reach targets satisfying the triangle inequality:
   $$|L_2 - L_3| \le D \le (L_2 + L_3)$$
   If $D > 21\text{ cm}$ or $D < 3\text{ cm}$, the target is outside the workspace and rejected.

4. **Solve Elbow Angle ($\theta_3$) via Law of Cosines**:
   Inside the triangle formed by $L_2$, $L_3$, and $D$:
   $$\cos\gamma = \frac{L_2^2 + L_3^2 - D^2}{2 L_2 L_3}$$
   $$\theta_3 = \text{acos}(\cos\gamma) \cdot \frac{180}{\pi}$$

5. **Solve Shoulder Angle ($\theta_2$)**:
   $$\alpha = \text{atan2}(z', r)$$
   $$\cos\beta = \frac{L_2^2 + D^2 - L_3^2}{2 L_2 D}$$
   $$\beta = \text{acos}(\cos\beta)$$
   $$\theta_2 = (\alpha + \beta) \cdot \frac{180}{\pi} \quad \text{(Elbow-Up configuration)}$$

---

## 4. Software File Structure

```
Robotics-Semester-3/
├── include/
│   ├── config.h            # Pin assignments, link lengths (4.5, 12.0, 9.0 cm), servo limits
│   ├── kinematics.h        # Vector3, Matrix4x4 definitions & Kinematics headers
│   └── arm_controller.h    # Servo PWM management, trajectory smoothing, demo
├── src/
│   ├── kinematics.cpp      # Mathematical implementation of FK, IK, and 4x4 matrices
│   ├── arm_controller.cpp  # Smooth multi-joint interpolation & pick-and-place
│   └── main.cpp            # Setup, command loop, and Serial interface
├── platformio.ini          # PlatformIO Uno configuration (115200 baud, Servo library)
└── README.md               # Complete project documentation & guide
```

---

## 5. Serial Monitor CLI Commands

To open the interactive terminal in PlatformIO, run:
```bash
pio device monitor -b 115200
```

### Available Commands:
| Command | Description | Example |
| :--- | :--- | :--- |
| `HELP` | Displays available commands and usage guide. | `HELP` |
| `HOME` | Smoothly moves arm back to home $(90^\circ, 90^\circ, 90^\circ)$. | `HOME` |
| `FK <b_deg> <s_deg> <e_deg>` | Moves arm to joint angles, prints the $4\times 4$ matrix & vector. | `FK 90 70 110` |
| `IK <x_cm> <y_cm> <z_cm>` | Solves IK for coordinate $(X, Y, Z)$ and smoothly navigates arm. | `IK 12.0 0.0 8.0` |
| `CLAMP <OPEN\|CLOSE>` | Opens ($100^\circ$) or closes ($30^\circ$) the clamp end-effector. | `CLAMP CLOSE` |
| `STATUS` | Prints current servo angles and live calculated end-effector vector. | `STATUS` |
| `DEMO` | Executes full 8-step Pick-and-Place sequence demonstrating FK & IK. | `DEMO` |
| `THEORY` | Prints linear algebra & robotics summary to the Serial monitor. | `THEORY` |

---

## 6. How to Build & Re-Upload

PlatformIO is pre-configured and the firmware has already been uploaded to your Arduino Uno (`/dev/cu.usbmodem141011`).

If you make modifications in the future:
1. **Build**:
   ```bash
   pio run
   ```
2. **Upload**:
   ```bash
   pio run -t upload
   ```
3. **Open Serial Console**:
   ```bash
   pio device monitor -b 115200
   ```
