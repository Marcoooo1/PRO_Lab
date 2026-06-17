# Probabilistic Robotics with ROS2 and TurtleBot4

**PRO Lab Project — Task 2510331007: Wrong Initialization**

Implementation and evaluation of three fundamental probabilistic state
estimation methods in mobile robotics — **Kalman Filter (KF)**, **Extended
Kalman Filter (EKF)**, and **Particle Filter (PF)** — on a simulated
TurtleBot4 in Gazebo, using ROS2 Jazzy.

**Core research question:** *If a filter is initialized with an incorrect
starting pose and/or uncertainty, is that error "dragged along" over time,
and how does this depend on the filter type and its assumed confidence?*

---

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Package Structure](#package-structure)
- [Building](#building)
- [Running a Simulation](#running-a-simulation)
- [Configuration Files](#configuration-files)
- [Evaluation / Plotting](#evaluation--plotting)
- [Experiments and Results](#experiments-and-results)
- [RViz Visualization](#rviz-visualization)
- [Known Limitations](#known-limitations)

---

## Overview

This project implements three ROS2 C++ nodes that estimate the 2D pose
(x, y, yaw) of a TurtleBot4 robot driving in a Gazebo simulation, using
wheel odometry as the control input and a (simulated) noisy pose
measurement as the observation:

| Filter | Node | Model |
|---|---|---|
| Kalman Filter | `kf_node` | Linear motion + linear measurement model |
| Extended Kalman Filter | `ekf_node` | Nonlinear differential-drive model, linearized via Jacobian |
| Particle Filter | `pf_node` | Sampling-based, 500 particles, systematic resampling |

All three filters run simultaneously on the same input data (odometry,
control commands) so their outputs can be directly compared against
ground truth and against each other.

The project specifically investigates **Wrong Initialization**: each
filter can be started with an incorrect initial pose estimate and/or an
incorrect initial uncertainty, to study whether and how strongly the
resulting error persists ("is dragged along") over time.

---

## System Requirements

- Ubuntu 24.04 (tested under WSL2)
- ROS2 Jazzy
- Gazebo Harmonic (`gz-harmonic`) via `turtlebot4_gz_bringup`
- Python 3 with `numpy`, `matplotlib` (for the offline evaluation script)

All filter, driver, and logging nodes are implemented in **C++**. Only
the offline evaluation script (`evaluate.py`) is Python, as it performs
no live ROS communication and is purely a post-processing tool.

---

## Package Structure

```
ros2_ws/src/
├── prob_robotics_filters/        # C++ ROS2 nodes: the three filters
│   ├── src/kf_node.cpp
│   ├── src/ekf_node.cpp
│   ├── src/pf_node.cpp
│   ├── src/landmark_detector.cpp
│   └── include/prob_robotics_filters/utils.hpp
│
├── prob_robotics_eval/            # Driving, logging, and evaluation
│   ├── src/auto_driver.cpp        # Drives the robot in a fixed pattern
│   ├── src/trajectory_logger.cpp  # Logs ground truth + all filter outputs to CSV
│   └── scripts/evaluate.py        # Offline RMSE computation + plotting
│
└── prob_robotics_bringup/         # Launch files and configuration
    ├── launch/all_filters.launch.py
    ├── config/filters_*.yaml      # One YAML per experiment scenario
    └── rviz/prob_robotics.rviz    # Pre-configured RViz display setup
```

---

## Building

```bash
cd ~/PROL/ros2_ws
colcon build
source install/setup.bash
```

To rebuild a single package after editing it:

```bash
colcon build --packages-select prob_robotics_filters
source install/setup.bash
```

---

## Running a Simulation

### Step 1 — Start the TurtleBot4 Gazebo simulation

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

Press **Play** (or spacebar) in the Gazebo GUI once it has loaded. The
robot starts docked at the origin.

### Step 2 — Launch the filters, driver, and logger

```bash
source ~/PROL/ros2_ws/install/setup.bash
ros2 launch prob_robotics_bringup all_filters.launch.py \
  config_file:=filters_correct_init.yaml \
  pattern:=circle \
  duration:=30.0 \
  csv_path:=/tmp/my_run.csv
```

This single launch file automatically:
1. Sends an `/undock` action goal and **waits for it to fully complete**
   (the TurtleBot4 ignores external velocity commands while docked or
   while an autonomous behavior, such as undocking, is in progress).
2. Starts `kf_node`, `ekf_node`, `pf_node`, and `trajectory_logger`.
3. Starts `auto_driver` only after the undock has genuinely finished,
   driving the requested pattern (`circle`, `straight`, …) for
   `duration` seconds.

**Important:** restart the Gazebo simulation (Step 1) before every run
to ensure the robot starts from the same docked position at the origin,
which is required for reproducible, comparable experiments.

| Argument | Default | Description |
|---|---|---|
| `config_file` | `filters_wrong_init.yaml` | YAML file from `prob_robotics_bringup/config/` |
| `pattern` | `circle` | Driving pattern for `auto_driver` |
| `duration` | `30.0` | Driving duration in seconds (after undocking) |
| `v` | `0.2` | Linear velocity [m/s] |
| `w` | `0.3` | Angular velocity [rad/s] |
| `csv_path` | `/tmp/trajectory_log.csv` | Output CSV path for the logger |

---

## Configuration Files

Each YAML file in `prob_robotics_bringup/config/` defines the parameters
for all three filter nodes for one experiment. Key parameters:

| Parameter | Filters | Meaning |
|---|---|---|
| `init_x`, `init_y`, `init_yaw` | KF, EKF, PF | Assumed initial pose |
| `init_cov_x/y/yaw` | KF, EKF | Initial covariance (confidence) |
| `init_spread_x/y/yaw` | PF | Initial particle cloud spread |
| `process_noise_q` | KF, EKF | Trust in the motion model |
| `process_noise_v`, `process_noise_w` | PF | Motion model noise (linear/angular) |
| `measurement_noise_r` | KF, EKF, PF | Trust in the position measurement |
| `measurement_noise_yaw` | PF | Trust in the heading measurement (decoupled from `measurement_noise_r`) |
| `num_particles` | PF | Number of particles |

Provided configuration files:

| File | Purpose |
|---|---|
| `filters_correct_init.yaml` | General-purpose baseline with correct initialization |
| `filters_W0_baseline.yaml` | Wrong-Init experiment: correct init (reference) |
| `filters_W2_large_offset.yaml` | Wrong-Init experiment: large pose offset, moderate confidence |
| `filters_W3_convinced_wrong.yaml` | Wrong-Init experiment: large offset, **high confidence in the wrong pose** |
| `filters_Q0.001/0.01/0.1/1.0.yaml` | Process noise (Q) variation, correct init |
| `filters_R0.001/0.05/0.5/2.0.yaml` | Measurement noise (R) variation, correct init |

---

## Evaluation / Plotting

After a run has produced a CSV log, evaluate it with:

```bash
ros2 run prob_robotics_eval evaluate.py /tmp/my_run.csv --tag my_run --out /tmp/plots
```

This prints an RMSE/convergence-time table to the console and writes two
plots to the output directory:

- `trajectories_<tag>.png` — ground truth vs. all filter trajectories in the XY plane
- `errors_<tag>.png` — position error, X error, and yaw error over time, per filter

**Convergence time (`t_conv`)** is defined as the first time at which the
position error drops below a threshold (default 0.2 m) and stays below
it for at least 2 seconds.

---

## Experiments and Results

Three categories of experiments were performed, in line with the
mandatory evaluation criteria of the lab (Q variation, R variation,
runtime, ground-truth RMSE) plus the assigned Wrong-Initialization task.
Full numeric results are documented in `results_summary.md`. Summary of
the key findings:

### 1. Wrong Initialization (core task)

| Filter | t_conv, correct init | t_conv, wrong init | t_conv, wrong init + overconfident |
|---|---|---|---|
| KF  | 1.00 s | 1.15 s | **5.10 s** |
| EKF | 1.00 s | 0.55 s | 2.65 s |
| PF  | 1.00 s | 25.15 s | 26.60 s |

**Finding:** the initialization error is dragged along more strongly the
more confident (lower initial uncertainty) the filter is in its wrong
belief. This effect is most pronounced in the Particle Filter, followed
by the classical Kalman Filter, while the EKF remains comparatively
robust due to its full nonlinear pose measurement update.

### 2. Process noise (Q) variation

The PF diverges completely at very low Q (RMSE > 2 m, particle cloud too
narrow to adapt to real motion deviations) and improves monotonically
with increasing Q. KF/EKF are far less sensitive to this parameter given
the low odometry noise in simulation.

### 3. Measurement noise (R) variation

KF/EKF show the expected monotonic degradation with increasing R. The PF
shows a clear sweet spot around R = 0.05: too tight an R causes particle
degeneration, too loose an R causes complete divergence (no convergence
within the test window at R = 2.0).

### 4. Runtime / Performance

All three filters reach a comparable update rate (~11–12 Hz, limited by
the odometry publish rate rather than computation time). Cumulative CPU
time increases in the expected order KF < EKF < PF, reflecting the
increasing model complexity (Jacobian computation for EKF; per-particle
likelihood evaluation and resampling for PF).

---

## RViz Visualization

A pre-configured RViz layout is provided:

```bash
rviz2 -d ~/PROL/ros2_ws/src/prob_robotics_bringup/rviz/prob_robotics.rviz
```

It displays, simultaneously:
- Ground truth pose (`/sim_ground_truth_pose`)
- KF, EKF, and PF pose estimates (`/kf/pose`, `/ekf/pose`, `/pf/pose`)
- The full PF particle cloud (`/pf/particles`)
- The defined landmark and its detection marker (`/landmark/marker`)

### Landmark Detector

A custom landmark is defined as a fixed point in the `odom` frame. The
`landmark_detector` node reports a (noisy) detection whenever the robot
is within a configurable range of that point:

```bash
ros2 run prob_robotics_filters landmark_detector --ros-args \
  -p landmark_x:=0.0 -p landmark_y:=0.0 -p detection_range:=1.0
```

---

## Known Limitations

- The Particle Filter is consistently the most sensitive of the three
  filters to parameter choice (initial spread, process noise, measurement
  noise) and to temporary control/measurement inconsistencies (e.g. brief
  periods where the simulated robot controller ignores velocity
  commands). This is treated as a genuine, documented finding rather than
  an unresolved bug — see `results_summary.md` for the full discussion.
- The simulated TurtleBot4 controller occasionally logs
  `"Ignoring velocities commanded while an autonomous behavior is
  running!"` for brief periods, even outside of docking/undocking. The
  filters' process noise was tuned to be robust against this, but it
  remains a source of variance between otherwise identical runs.
- All experiments use a `circle` driving pattern at a fixed, deliberately
  low speed (v = 0.2 m/s, w = 0.3 rad/s) for safety and reproducibility
  in the simulated warehouse environment.
