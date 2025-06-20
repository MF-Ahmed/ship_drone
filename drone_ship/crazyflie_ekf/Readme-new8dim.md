# Sensor Fusion for UAV Localization Using Extended Kalman Filter (EKF)

## 1. Introduction

In autonomous drone navigation, accurate localization is crucial. Relying on a single sensor like GPS or an IMU can lead to significant errors due to noise, latency, or drift. Therefore, sensor fusion techniques—specifically the **Extended Kalman Filter (EKF)**—are employed to combine information from multiple sources to produce a more reliable state estimate. In this study, we fuse **GPS**, **IMU**, and optionally **odometry** data to estimate the drone's 3D position and velocity.

---

## 2. State Definition

We define the state vector as:

\[
\mathbf{x} =
\begin{bmatrix}
x & y & z & v_x & v_y & v_z & b_{ax} & b_{ay}
\end{bmatrix}^T
\]

Where:
- \(x, y, z\): position in East-North-Up (ENU) frame
- \(v_x, v_y, v_z\): velocity
- \(b_{ax}, b_{ay}\): accelerometer bias (for x and y axes)

---

## 3. System Dynamics (Prediction Model)

Assuming a **constant acceleration model**, the dynamics are given by:

\[
\mathbf{x}_{k} = \mathbf{F}_k \mathbf{x}_{k-1} + \mathbf{B}_k \mathbf{u}_k + \mathbf{w}_k
\]

Where:
- \(\mathbf{u}_k = \begin{bmatrix} a_x & a_y & a_z \end{bmatrix}^T\) are acceleration inputs from the IMU, corrected for gravity and bias
- \(\mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q})\) is process noise

### Transition Matrix \( \mathbf{F}_k \):

\[
\mathbf{F}_k =
\begin{bmatrix}
\mathbf{I}_3 & \Delta t \cdot \mathbf{I}_3 & \mathbf{0}_{3 \times 2} \\
\mathbf{0} & \mathbf{I}_3 & -\Delta t \cdot \mathbf{I}_{2 \times 2} \\
\mathbf{0}_{2 \times 3} & \mathbf{0}_{2 \times 3} & \mathbf{I}_2
\end{bmatrix}
\]

- The bias terms influence velocity evolution, modeled as a disturbance.

### Control Matrix \( \mathbf{B}_k \):

\[
\mathbf{B}_k =
\begin{bmatrix}
\frac{1}{2} \Delta t^2 \cdot \mathbf{I}_3 \\
\Delta t \cdot \mathbf{I}_3 \\
\mathbf{0}_{2 \times 3}
\end{bmatrix}
\]

---

## 4. Measurement Model (GPS)

GPS provides position observations \( \mathbf{z}_k = \begin{bmatrix} x_{gps}, y_{gps}, z_{gps} \end{bmatrix}^T \). The observation equation is:

\[
\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k
\]

Where:
- \( \mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R}) \) is measurement noise
- \( \mathbf{H} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0}_{3 \times 5} \end{bmatrix} \)

### Measurement Noise Covariance:

\[
\mathbf{R} = \text{diag}(\sigma_{xy}^2, \sigma_{xy}^2, \sigma_{z}^2)
\]

Larger noise is assigned to the vertical axis due to lower GPS accuracy in altitude.

---

## 5. EKF Algorithm

### Prediction Step:

\[
\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_k \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{B}_k \mathbf{u}_k
\]
\[
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^\top + \mathbf{Q}
\]

### Update Step:

\[
\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1} \quad \text{(Innovation)}
\]
\[
\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^\top + \mathbf{R}
\]
\[
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^\top \mathbf{S}_k^{-1}
\]
\[
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
\]
\[
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}
\]

---

## 6. Bias Estimation and Compensation

To prevent **lateral drift** caused by IMU bias, the EKF estimates \( b_{ax}, b_{ay} \) over time. These biases are subtracted from raw IMU acceleration before integration:

\[
a_x^{\text{corrected}} = a_x^{\text{imu}} - b_{ax}
\quad\quad
a_y^{\text{corrected}} = a_y^{\text{imu}} - b_{ay}
\]

This correction helps reduce velocity and position drift, especially in the XY plane.

---

## 7. Practical Considerations

- GPS readings are converted from geographic (lat, lon, alt) to ENU using a fixed reference.
- Outlier rejection is implemented to ignore sudden GPS jumps.
- The state estimate is published as a `nav_msgs/Odometry` message with position, velocity, and IMU orientation.
- Transform between `odom` and `base_link` is published using `tf2`.

---

## 8. Linearization Note

In this implementation, both motion and observation models are **linear**. Therefore, the Jacobians \( \mathbf{F}_k \) and \( \mathbf{H} \) are exact. If future sensors or models are nonlinear, linearization via Jacobians will be required.

---

## 9. Conclusion

This EKF implementation enables robust, drift-resistant 3D localization for drones in outdoor GPS environments. It supports bias estimation, real-time velocity correction, and provides a framework for incorporating more sensors like barometers or vision systems.

