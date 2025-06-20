## EKF State Transition and Control Matrices

### ✅ State Vector

The 8-dimensional EKF state vector is defined as:

\[
\mathbf{x} = 
\begin{bmatrix}
x \\
y \\
z \\
v_x \\
v_y \\
v_z \\
b_{ax} \\
b_{ay}
\end{bmatrix}
\]

- \(x, y, z\): Position in ENU frame  
- \(v_x, v_y, v_z\): Linear velocities  
- \(b_{ax}, b_{ay}\): Accelerometer biases (x and y)

---

### ✅ Motion Model

Assuming constant acceleration and small time step \(\Delta t\), the motion model is:

\[
\mathbf{x}_k = \mathbf{F}_k \mathbf{x}_{k-1} + \mathbf{B}_k \mathbf{u}_{k-1} + \mathbf{w}_k
\]

Where:  
- \(\mathbf{u}_{k-1}\) is the bias-corrected acceleration from IMU: \([a_x, a_y, a_z]^T\)  
- \(\mathbf{w}_k\) is process noise

---

### ✅ State Transition Matrix \( \mathbf{F}_k \)

\[
\mathbf{F}_k =
\begin{bmatrix}
1 & 0 & 0 & \Delta t & 0       & 0       & 0         & 0         \\
0 & 1 & 0 & 0        & \Delta t & 0       & 0         & 0         \\
0 & 0 & 1 & 0        & 0        & \Delta t & 0         & 0         \\
0 & 0 & 0 & 1        & 0        & 0       & -\Delta t & 0         \\
0 & 0 & 0 & 0        & 1        & 0       & 0         & -\Delta t \\
0 & 0 & 0 & 0        & 0        & 1       & 0         & 0         \\
0 & 0 & 0 & 0        & 0        & 0       & 1         & 0         \\
0 & 0 & 0 & 0        & 0        & 0       & 0         & 1
\end{bmatrix}
\]

Explanation:
- Positions \(x, y, z\) evolve with velocity
- Velocities \(v_x, v_y\) are affected by biases \(b_{ax}, b_{ay}\)

---

### ✅ Control Input Matrix \( \mathbf{B}_k \)

\[
\mathbf{B}_k =
\begin{bmatrix}
\frac{1}{2} \Delta t^2 & 0                 & 0 \\
0                 & \frac{1}{2} \Delta t^2 & 0 \\
0                 & 0                 & \frac{1}{2} \Delta t^2 \\
\Delta t          & 0                 & 0 \\
0                 & \Delta t          & 0 \\
0                 & 0                 & \Delta t \\
0                 & 0                 & 0 \\
0                 & 0                 & 0
\end{bmatrix}
\]

Explanation:
- IMU accelerations influence velocity and position
- No control influence on the bias states

---

### ✅ Summary

- \(\mathbf{F}_k\) handles state propagation considering velocity and bias
- \(\mathbf{B}_k\) maps accelerometer readings to position and velocity changes
- Accelerometer biases are estimated over time and help correct drift

This model enables robust and drift-limited position estimation using only GPS and IMU in an EKF framework.
