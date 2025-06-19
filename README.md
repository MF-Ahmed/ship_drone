# Extended Kalman Filter (EKF) for Sensor Fusion of GPS, IMU, and Odometry Data

## 1. Introduction

Sensor fusion is critical for accurate localization and navigation of autonomous vehicles, particularly in environments where individual sensors exhibit limitations. In this work, we implement an Extended Kalman Filter (EKF) to fuse data from an Inertial Measurement Unit (IMU), a Global Positioning System (GPS) receiver, and derived odometry. The aim is to estimate the 3D position and velocity of a Crazyflie drone in an Earth-fixed East-North-Up (ENU) coordinate frame.

## 2. System Model

We define the system state vector as:

$$
\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z \end{bmatrix}^\top

$$

where \(x, y, z\) represent position, and \(v_x, v_y, v_z\) represent linear velocity in ENU.

The system follows the standard constant-velocity model:

$$
\mathbf{x}_k = \mathbf{F}_k \mathbf{x}_{k-1} + \mathbf{B}_k \mathbf{u}_k + \mathbf{w}_k

$$

where:

- \(\mathbf{F}_k\) is the state transition matrix,
- \(\mathbf{u}_k\) is the control input (acceleration from IMU),
- \(\mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q})\) is the process noise.

The matrices are defined as:

$$
\mathbf{F}_k =
\begin{bmatrix}
\mathbf{I}_3 & \Delta t \cdot \mathbf{I}_3 \\
\mathbf{0} & \mathbf{I}_3
\end{bmatrix}, \quad
\mathbf{B}_k =
\begin{bmatrix}
0.5 \cdot \Delta t^2 \cdot \mathbf{I}_3 \\
\Delta t \cdot \mathbf{I}_3
\end{bmatrix}

$$

where \(\Delta t\) is the time step.

## 3. Measurement Model

The GPS provides absolute position:

$$
\mathbf{z}_{GPS} = \begin{bmatrix} x_{GPS} & y_{GPS} & z_{GPS} \end{bmatrix}^\top

$$

The measurement model is:

$$
\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k \quad \text{with} \quad
\mathbf{H} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0} \end{bmatrix}

$$

where \(\mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R})\) is the measurement noise.

Measurement noise covariance:

$$
\mathbf{R} = \text{diag}(\sigma_{xy}^2, \sigma_{xy}^2, \sigma_{z}^2)

$$

Note: Vertical noise \(\sigma_z\) is larger due to GPS altitude uncertainty.

## 4. EKF Algorithm

### Prediction Step

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_k \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{B}_k \mathbf{u}_k

$$

$$
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^\top + \mathbf{Q}

$$

### Update Step (GPS)

$$
\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}

$$

$$
\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^\top + \mathbf{R}

$$

$$
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^\top \mathbf{S}_k^{-1}

$$

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k

$$

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}

$$

## 5. IMU and GPS Integration

- IMU acceleration is used as input \(\mathbf{u}_k\).
- Gravity compensation: \(a_z^{\text{corrected}} = a_z - 9.81\)
- GPS data is converted to ENU frame using a known reference point.
- Sudden GPS jumps are discarded using an ENU distance threshold.

## 6. Tuning and Gain Adjustment

- Kalman Gain \(\mathbf{K}_k\) balances trust between model and measurement.
- Tuning parameters:
  - \(\mathbf{Q}\): Higher value increases model uncertainty (trusts GPS more).
  - \(\mathbf{R}\): High \(\sigma_z\) reduces influence of noisy GPS altitude.

## 7. Output

The EKF publishes fused estimates as `nav_msgs/Odometry`:

- 3D Position
- 3D Velocity
- Orientation (from IMU)
- TF transform between odom and base link

## 8. Conclusion

The EKF fuses fast IMU data with slower GPS updates to provide accurate localization. This method is robust to noise and drift, and can be extended to include more sensors like barometers or visual odometry.

## 9. Linearization Note

As the motion and measurement models are linear, no Jacobians are needed. For nonlinear systems, linearization using:

$$
\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}}, \quad \mathbf{H}_k = \frac{\partial h}{\partial \mathbf{x}}

$$

would be required.

## 6. Linearization in Extended Kalman Filter

The Extended Kalman Filter (EKF) extends the standard Kalman Filter to handle **nonlinear systems** by linearizing the system dynamics and measurement models around the current estimate.

Let the nonlinear state transition and measurement models be:

- **Process model**:
  $$
  \mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k

  $$
- **Measurement model**:
  $$
  \mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k

  $$

To apply Kalman filtering, we **linearize** these functions using their **Jacobians**:

- **State transition Jacobian**:
  $$
  \mathbf{F}_k = \left. \frac{\partial f}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k}

  $$
- **Control input Jacobian**:
  $$
  \mathbf{B}_k = \left. \frac{\partial f}{\partial \mathbf{u}} \right|_{\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k}

  $$
- **Measurement Jacobian**:
  $$
  \mathbf{H}_k = \left. \frac{\partial h}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k|k-1}}

  $$

---

## 7. Derivation of \( \mathbf {F}_k \) and \( \mathbf {B}_k \) for Constant Acceleration Model

We use a **constant-acceleration motion model**, where the state vector is:

$$
\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z \end{bmatrix}^\top

$$

The control input is the **IMU acceleration**:

$$
\mathbf{u} = \begin{bmatrix} a_x & a_y & a_z \end{bmatrix}^\top

$$

---

### 7.1 Discrete-Time State Update Equations

Using Newtonian kinematics:

- Position update:

  $$
  \mathbf{p}_{k} = \mathbf{p}_{k-1} + \Delta t \cdot \mathbf{v}_{k-1} + \frac{1}{2} \Delta t^2 \cdot \mathbf{a}_k

  $$
- Velocity update:

  $$
  \mathbf{v}_k = \mathbf{v}_{k-1} + \Delta t \cdot \mathbf{a}_k

  $$

---

### 7.2 Constructing \( \mathbf {F}_k \) and \( \mathbf {B}_k \)

The state transition matrix \( \mathbf{F}_k \) captures how the previous state affects the current state without input:

$$
\mathbf{F}_k =
\begin{bmatrix}
\mathbf{I}_3 & \Delta t \cdot \mathbf{I}_3 \\
\mathbf{0}_3 & \mathbf{I}_3
\end{bmatrix}

$$

The control input matrix \( \mathbf{B}_k \) describes how acceleration influences the position and velocity:

$$
\mathbf{B}_k =
\begin{bmatrix}
0.5 \cdot \Delta t^2 \cdot \mathbf{I}_3 \\
\Delta t \cdot \mathbf{I}_3
\end{bmatrix}

$$

where:

- \( \mathbf{I}_3 \): 3×3 identity matrix
- \( \mathbf{0}_3 \): 3×3 zero matrix
- \( \Delta t \): time interval between updates

---

### 7.3 Final Linearized Model

The linearized prediction equation used in EKF becomes:

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_k \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{B}_k \mathbf{u}_k

$$

The covariance is propagated as:

$$
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^\top + \mathbf{Q}

$$

---

## Summary

- Linearization enables the EKF to operate on nonlinear systems by approximating them locally with linear models.
- The Jacobians \( \mathbf{F}_k \), \( \mathbf{B}_k \), and \( \mathbf{H}_k \) are derived from partial derivatives of the system and measurement functions.
- For a constant-acceleration motion model, the linearized matrices take a convenient analytical form that integrates acceleration into the position and velocity estimates.
