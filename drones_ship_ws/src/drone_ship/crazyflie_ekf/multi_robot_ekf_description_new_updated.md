
# Extended Kalman Filter for Multi-Robot State Estimation

## Overview

This project implements an Extended Kalman Filter (EKF) for estimating the state (position and velocity) of multiple autonomous robots (1 surface Aquabot and 3 aerial drones) using GPS and IMU measurements in a ROS 2 simulation.

## EKF State Vector

The EKF uses an 8-dimensional state vector:

\[
\mathbf{x}_k = \begin{bmatrix}
x & y & z & v_x & v_y & v_z & b_{ax} & b_{ay}
\end{bmatrix}^T
\]

Where:
- \(x, y, z\): Position in ENU frame
- \(v_x, v_y, v_z\): Velocity
- \(b_{ax}, b_{ay}\): Accelerometer bias (for x and y axes)

## Prediction Model

The prediction step uses IMU accelerometer data to propagate the state:

### State Transition Equation

\[
\mathbf{x}_{k|k-1} = \mathbf{F}_k \mathbf{x}_{k-1|k-1} + \mathbf{B}_k \mathbf{u}_k
\]

Where:
- \(\mathbf{F}_k\): State transition matrix
- \(\mathbf{B}_k\): Control input matrix
- \(\mathbf{u}_k = [a_x, a_y, a_z]^T\): IMU linear acceleration (gravity compensated)

### State Transition Matrix \(\mathbf{F}_k\):

\[
\mathbf{F}_k = \begin{bmatrix}
1 & 0 & 0 & \Delta t & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 & -\Delta t & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & -\Delta t \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
\]

### Control Input Matrix \(\mathbf{B}_k\):

\[
\mathbf{B}_k = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
\Delta t & 0 & 0 \\
0 & \Delta t & 0 \\
0 & 0 & \Delta t \\
0 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\]

### Process Noise Covariance \(\mathbf{Q}_k\):

\[
\mathbf{Q}_k = \begin{bmatrix}
0 & & & & & & & \\
& 0 & & & & & & \\
& & 0 & & & & & \\
& & & \sigma_v^2 & & & & \\
& & & & \sigma_v^2 & & & \\
& & & & & \sigma_v^2 & & \\
& & & & & & \sigma_{bax}^2 & \\
& & & & & & & \sigma_{bay}^2
\end{bmatrix}
\]

## Measurement Model

GPS provides absolute position measurements in ENU coordinates.

### Measurement Equation:

\[
\mathbf{z}_k = \mathbf{H}_k \mathbf{x}_{k|k-1} + \mathbf{w}_k
\]

Where \(\mathbf{H}_k\) is the observation matrix:

\[
\mathbf{H}_k = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\]

And \(\mathbf{w}_k\) is measurement noise with covariance \(\mathbf{R}_k\):

\[
\mathbf{R}_k = \text{diag}(\sigma_{gps,x}^2, \sigma_{gps,y}^2, \sigma_{gps,z}^2)
\]

## Update Step

Compute innovation:

\[
\mathbf{y}_k = \mathbf{z}_k - \mathbf{H}_k \mathbf{x}_{k|k-1}
\]

Compute Kalman gain:

\[
\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k \\
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}
\]

Update state estimate:

\[
\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
\]

Update covariance:

\[
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1}
\]

## Multi-Robot Generalization

Each robot (drone1, drone2, drone3, aquabot) runs an independent instance of the EKF. Each EKF uses namespaced topics (`/droneX/imu`, `/droneX/gps`) and shares the same state estimation logic. This enables distributed, synchronized state estimation in a decentralized setup.

## Diagram

![EKF Timeline](A_diagram_illustrates_the_timeline_and_steps_of_th.png)
