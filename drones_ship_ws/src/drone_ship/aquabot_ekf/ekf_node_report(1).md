# Extended Kalman Filter (EKF) Node in ROS 2

## 🧭 Purpose

This ROS 2 C++ node implements a 6-state Extended Kalman Filter (EKF) to estimate the **3D position and velocity** of a mobile robot or drone using data from GPS and IMU sensors. It publishes filtered odometry and broadcasts coordinate transforms for use in localization and control.

---

## 🧠 State Vector

The EKF estimates a 6D state:

```
x_ = [x, y, z, vx, vy, vz]
```

Where:
- `x, y, z` = position in ENU frame (meters)
- `vx, vy, vz` = linear velocity in ENU (m/s)

---

## ⚙️ Parameters

The node is configurable using ROS 2 parameters:

| Parameter               | Description                                  | Default         |
|------------------------|----------------------------------------------|-----------------|
| `q_pos`                | Process noise for position                    | `0.01`          |
| `q_vel`                | Process noise for velocity                    | `0.01`          |
| `r_gps_x/y/z`          | GPS measurement noise                         | `5.0`           |
| `gps_x/y_offset`       | Optional manual offsets for calibration       | `0.0`           |
| `initial_state`        | Initial state vector                          | `[0, 0, 0, 0, 0, 0]` |
| `utm_origin_*`         | UTM coordinates for origin (if static)        | `0.0`           |

---

## 🔄 EKF Overview

The EKF has two key stages:

### 1. Prediction (IMU-based)
- IMU linear acceleration is integrated to update velocity and then position.
- Gravity is subtracted from Z-axis acceleration.
- Covariance is propagated using a discrete-time linear motion model.

### 2. Update (GPS-based)
- GPS lat/lon is converted to local ENU using UTM projection.
- The ENU position is used to update the EKF using the Kalman gain.

---

## 🛰️ GPS Processing

When a valid GPS fix is received:
1. The coordinates are converted to **UTM** using GeographicLib.
2. The origin is dynamically set on the first fix.
3. Local ENU coordinates are calculated:

```
local_x = easting - origin_easting - gps_x_offset
local_y = northing - origin_northing - gps_y_offset
local_z = altitude - origin_altitude
```

4. Kalman filter update step:
```
z = measured position
z_pred = x_.segment<3>(0)
y = z - z_pred
K = P Hᵀ (H P Hᵀ + R)⁻¹
x_ = x_ + K y
P = (I - K H) P
```

---

## 🧲 IMU Prediction

Triggered on every IMU message:

1. Calculate `dt` between messages.
2. Integrate linear acceleration to update velocity and position.
3. Use Jacobian `F` to propagate the error covariance `P`:

```
F = identity(6x6)
F(0,3) = dt; F(1,4) = dt; F(2,5) = dt
P = F P Fᵀ + Q
```

---

## 🚀 Publishing

The node publishes:

### `/odometry/filtered` (nav_msgs/Odometry)
- `pose`: filtered position and IMU orientation
- `twist`: estimated linear velocity

### `TF` transform
- From `aquabot/odom` to `aquabot/base_link` for RViz and control frames.

---

## 🔧 Topics

### Subscribed:
- `/aquabot/sensors/imu/imu/data` → `sensor_msgs/msg/Imu`
- `/aquabot/sensors/gps/gps/fix` → `sensor_msgs/msg/NavSatFix`

### Published:
- `/odometry/filtered` → `nav_msgs/msg/Odometry`
- TF → `geometry_msgs/msg/TransformStamped`

---

## 📌 Usage

1. Add your EKF node to a ROS 2 launch file.
2. Set parameters in a `.yaml` or directly in the launch file.
3. Start the simulation or bring up sensors.
4. Visualize `/odometry/filtered` and TF tree in RViz.

---

## ✅ Benefits

- Smooths GPS noise using IMU integration.
- Robust in outdoor environments where GPS is intermittent.
- Parameterized and modular — easy to extend or tune.

---

## 🔄 Possible Extensions

- Add **barometer** for better Z estimation.
- Fuse **visual odometry** or **wheel odometry**.
- Add orientation tracking using quaternion + gyroscope integration.
- Estimate IMU bias terms for better accuracy.

---

## 📚 Conclusion

This EKF node provides a solid baseline for fusing GPS and IMU data into a coherent pose and velocity estimate. It is lightweight, extensible, and tailored for mobile robotics applications like drones or marine vehicles operating in outdoor environments.---

## 🧮 Detailed EKF Matrix Computation

### 📐 State Vector

The 6D state vector is defined as:

```
x = [x, y, z, vx, vy, vz]
```

- Position: (x, y, z) in local ENU
- Velocity: (vx, vy, vz) in local ENU

---

### 🔁 Process Model (Prediction)

#### Motion model used:
```
x_pos += vx * dt
y_pos += vy * dt
z_pos += vz * dt

vx += ax * dt
vy += ay * dt
vz += (az - g) * dt
```

- `ax, ay, az`: Linear acceleration from IMU
- `g`: Gravity (9.81 m/s²) subtracted from vertical accel

#### Jacobian of the Process Model (F)

```
F = d(f)/d(x) =
[[1, 0, 0, dt, 0,  0],
 [0, 1, 0, 0,  dt, 0],
 [0, 0, 1, 0,  0,  dt],
 [0, 0, 0, 1,  0,  0],
 [0, 0, 0, 0,  1,  0],
 [0, 0, 0, 0,  0,  1]]
```

This matrix is time-dependent and is recomputed at each step using the current `dt`.

---

### 🛰️ Measurement Model (GPS Update)

#### Observation model:
The GPS provides direct measurements of position only.

```
z = H * x + noise
```

#### Jacobian of Measurement Model (H)

```
H =
[[1, 0, 0, 0, 0, 0],
 [0, 1, 0, 0, 0, 0],
 [0, 0, 1, 0, 0, 0]]
```

This matrix maps the state to the observed variables (position).

---

### 📋 Summary

| Matrix | Size | Purpose                        | Description                    |
|--------|------|--------------------------------|--------------------------------|
| F      | 6x6  | Motion model Jacobian          | Time-dependent, linear         |
| H      | 3x6  | Measurement model Jacobian     | Constant, position-only update |

These matrices are crucial for propagating and correcting state estimates in the EKF.