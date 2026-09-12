# Decentralized Multi-Robot Obstacle Detection and Tracking in a Maritime Scenario


> Autonomous aerial–surface robot teams are promising for maritime monitoring. This work
> presents a decentralized multi-robot framework for detecting and tracking floating containers
> using multiple UAVs cooperating with an autonomous surface vessel (Aquabot). Each UAV performs
> YOLOv8-based visual detection and stereo-disparity 3D lifting, then tracks targets with
> per-object EKFs using uncertainty-aware data association. Compact track summaries are
> exchanged and fused conservatively via covariance intersection, ensuring consistency under
> unknown correlations. An information-driven assignment module allocates targets and selects
> UAV hover viewpoints by trading expected uncertainty reduction against travel effort and
> safety constraints.

This repository contains the ROS 2/Gazebo simulation that produces the paper's results (Section
IV). 

## Repository layout

- `README.md` — this file.
- `main.pdf` — the paper.
- `drones_ship_ws/` — the ROS 2 colcon workspace (`src/drone_ship/...`); all the code lives here.
- `models/` — standalone Gazebo model assets (meshes/materials for containers, barrels, logs,
  a vessel) used as simulated floating obstacles. Not part of the colcon workspace itself, but
  required at runtime (see [Obstacle models](#obstacle-models-one-time-setup) below).

## Method overview

The system (Fig. 1 of the paper) is a closed loop of five stages, run independently on each UAV
plus one shared assignment step:

1. **Agent localization** — each UAV (and the Aquabot) runs its own onboard EKF fusing GPS and
   IMU into odometry in a shared ENU world frame; no robot depends on another's state estimate.
2. **Detection** — each UAV runs YOLOv8 on its onboard stereo camera and lifts 2D detections to
   metric 3D positions using stereo disparity.
3. **Local tracking** — each 3D detection is associated to existing per-object EKF tracks using
   Mahalanobis-distance gating, so every UAV maintains its own persistent set of container track
   hypotheses; tracks that go unobserved for too long are pruned.
4. **Decentralized fusion** — UAVs exchange only compact track summaries (mean + covariance, no
   raw detections), which are combined into a shared fused-track set via Covariance Intersection
   (CI). CI makes no assumption about cross-robot correlations, so fusing tracks that share
   information (e.g. through common priors) never produces an overconfident result.
5. **Assignment and hover selection** — an allocator poses target assignment as a capacitated
   min-cost flow (CMCF) problem, balancing expected uncertainty reduction (information gain)
   against travel distance, inter-robot separation, and assignment stickiness (avoiding
   unnecessary target switching). For each UAV assigned a target, a hover viewpoint is chosen
   from a discrete ring of candidates around it by maximizing expected information gain
   (D-optimality). Each UAV switches between a `SURVEILLANCE` mode (no active assignment) and a
   `TRACKING` mode (hovering near its assigned target), handing a target back to `SURVEILLANCE`
   once it is well localized or no longer worth observing.

The method is compared in simulation against YOLOv8-RGB-only detection, DeepSORT and AB3DMOT
tracking baselines, and centralized/independent (non-CI) fusion, using COCO-style detection
metrics (mAP, precision/recall), MOT metrics (MOTA, ID switches, fragmentation), 3D localization
RMSE, and fusion consistency/coverage/communication-overhead metrics.

Per-package design notes (state definitions, tuning) live alongside the code:
- `drones_ship_ws/src/drone_ship/crazyflie_ekf/Readme-new8dim.md` and `Readme-new8dim_FandB.md`
  — the per-drone localization EKF actually implemented (8-state, with accelerometer bias).
  `Readme.md` in the same folder documents an earlier, simpler 6-state version and is superseded.
- `drones_ship_ws/src/drone_ship/aquabot_ekf` — the Aquabot's own EKF (6-state, no bias, UTM
  projection instead of the drones' ENU projection).

## Dependencies

Developed and tested on **Ubuntu with ROS 2 Jazzy + Gazebo Harmonic**. Everything below assumes
that combination; other ROS 2/Gazebo pairings are not supported by this codebase.

**Core platform**
- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) + the `ros_gz` bridge packages
  (`ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-interfaces`)
- `colcon` (`python3-colcon-common-extensions`)

**ROS 2 package dependencies** (apt, `ros-jazzy-<name>`): `xacro`, `joint-state-publisher-gui`,
`sdformat-urdf`, `tf2-ros`, `tf2-geometry-msgs`, `cv-bridge`, `message-filters`,
`image-transport`, `vision-msgs`, `stereo-msgs`, `visualization-msgs`, `rosidl-default-runtime`.
Easiest way to pull these in bulk from the workspace root:
```bash
cd drones_ship_ws
rosdep install --from-paths src --ignore-src -r -y
```
(`rosdep` won't fully resolve the vendored VRX/Aquabot packages, whose `package.xml` maintainer
fields are unfilled competition-kit placeholders — install anything it reports as unresolved
from the apt list above.)

**System/C++ libraries**: `libeigen3-dev`, `libyaml-cpp-dev`, `libgeographic-dev` (used by
`aquabot_ekf` for UTM projection via GeographicLib), `libopencv-dev`.

**Python (pip)** — used by the perception scripts and the offline evaluation scripts:
```bash
pip install ultralytics opencv-python numpy pandas matplotlib scipy
```
(`ultralytics` provides YOLOv8, used by `crazyflie_yolo`'s detector node; `scipy` is used by
`aquabot_motion`'s path planner; `pandas`/`matplotlib` are only needed to run the `metrics/`
summary scripts, not the simulation itself.)

There is no `requirements.txt`/`setup.cfg` pinning versions anywhere in the repo — the above is
reconstructed from actual imports, so pin versions yourself if you need reproducibility.

## Build

```bash
cd drones_ship_ws
colcon build --symlink-install
source install/setup.zsh   # or setup.bash
```
Recommended environment (see `.zshrc` at the repo root for the full reference shell setup):
```bash
export GZ_VERSION=harmonic
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=111       # pick any value shared by all your terminals
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gz/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/.gz/models
```
Note: several launch files hard-code the absolute path `/home/user/data/drones_ship_ws/...`
(for xacro/URDF/model files and for the YOLO weights path — see below). If your workspace lives
somewhere else, update those paths in the relevant `launch/*.py` files, or clone/symlink the
workspace to that exact path.

### Obstacle models (one-time setup)

The main simulation launch spawns floating obstacles (`container1`–`container5`, `barrel1`) by
loading `~/.gz/models/newnames/<name>/model.sdf` — **these are not looked up from inside the
colcon workspace**, so without this step the world will start without any obstacles for the
drones to detect. Copy them into place once:
```bash
mkdir -p ~/.gz/models
cp -r "models/newnames" ~/.gz/models/
```
(The rest of the top-level `models/` directory — `aniskm-vessel-e`, `floating_container`,
`greyboxlarge`, `log1`, `log2`, `barrel2`, etc. — holds additional obstacle assets that exist in
the repo but are currently commented out / unused by the active launch files.)

## Running the simulation

There is no single umbrella launch file — the full pipeline is brought up as a sequence of
`ros2 launch`/`ros2 run` commands, normally one per terminal, in this order:

```bash
# 1. World + ship + 3 Crazyflie drones + obstacle spawns, in Gazebo
ros2 launch aquabot_gz full_system_launch.py gui:=False

# 2. Ship (USV) EKF
ros2 launch aquabot_ekf ekf_launch.py

# 3. Per-drone EKFs (8-state GPS/IMU fusion)
ros2 launch crazyflie_ekf all_drones_ekf_new.launch.py drones:=drone1,drone2,drone3

# 4. Per-drone action servers (explore/track/navigate/move)
ros2 launch crazyflie_servers all_drone_explore_track_servers_new.launch.py drones:=drone1,drone2,drone3

# 5. Kick a drone off on an initial hover + forward move
ros2 run crazyflie_servers dual_action_client --ros-args \
  -p ns:=drone1 -p target_z:=10.0 -p hover_sec:=2 -p yaw:=0.0 \
  -p distance_x:=8.0 -p distance_y:=-8.0 -p speed:=0.20
# (repeat for ns:=drone2, ns:=drone3 with different distance_x/distance_y)

# 6. Stereo YOLO detection + per-drone 3D tracking
ros2 launch crazyflie_yolo full_system_yolo_stereo_track.launch.py

# 7. Decentralized fusion (covariance intersection) + CMCF target assignment, with CSV logging
ros2 launch crazyflie_yolo full_system_fusion_hungarian.launch.py \
  fusion_log_path:=<path/to/fused_obstacles.csv> \
  event_log_path:=<path/to/fusion_events.csv> \
  prune_log_path:=<path/to/prune_events.csv> \
  stats_log_path:=<path/to/fusion_stats.csv> \
  assignment_csv_path:=<path/to/assignment_log.csv>
```
(The launch file above is named `full_system_fusion_hungarian.launch.py` for historical reasons —
the assignment it starts is the paper's capacitated min-cost-flow allocator, not a plain
Hungarian/bipartite matcher.)

Step 6 already loads a pre-trained YOLOv8 checkpoint committed in the repo
(`crazyflie_yolo/dataset_rgb/runs/yolo8_rgb/train4/weights/best.pt`) — you don't need to train a
model to run inference; the `yolov8n.pt` checkpoint at the workspace root is the base weights
used for training that model, not what's loaded at runtime.

### Convenience scripts

`drones_ship_ws/Simulation_env_ekfs.py` and `Servers_track_fusion_assignment_{first,second}.py`
automate steps above by opening timed `gnome-terminal` tabs running the same commands (with a
couple of alternate drone flight-path parameters between the `_first`/`_second` variants). They
assume a Linux desktop with `gnome-terminal` and `zsh`, and the hard-coded workspace path
mentioned above — treat them as a recorded reference for the bring-up sequence/timing rather
than portable tooling.

### Shutting down

```bash
drones_ship_ws/bin/killgz_services
```
Requests a clean Gazebo shutdown, then force-kills any leftover `gz sim`/`gz-gui`/
`ros_gz_bridge`/`parameter_bridge` processes.

## Evaluation / metrics

The tracking and fusion nodes above write CSV logs (see `crazyflie_yolo/logs_fusion/`,
`logs_assignment/`, and the root `metrics/*.csv`). Two standalone scripts turn those into the
paper's summary tables and figures:
```bash
cd drones_ship_ws/metrics
python3 summarize_tracking_metrics.py --files drone1_tracking_metrics.csv drone2_tracking_metrics.csv drone3_tracking_metrics.csv
python3 summarize_fusion_metrics.py --csv fusion_metrics.csv
```
Both accept `--help` for their full set of output-path options (they write a summary CSV, a
rendered JPG table, and — for tracking — an IEEE-style LaTeX table snippet).

## Testing

There is no functional test suite in the project-original packages — only
`ament_lint_auto`/`ament_lint_common` style/copyright linting wired into `colcon test`. Validate
changes by running the simulation and/or the metrics scripts above.
