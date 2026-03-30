# Particle Filter Localization

## Academic Year

2025/2026

## Build the project

### Requirements

> [!NOTE]
> **This is the general setup for installing ROS2 and testing on linux distros. In my case I use MacOS and the install is detailed in the repo README**
>

- ROS2 (command depends on your Linux distribution)
- Example for Ubuntu 24:

```bash
sudo apt install libpcl-dev ros-jazzy-pcl-conversions ros-jazzy-pcl-msgs
```

### ROS bag

Download the log file here:
<https://drive.google.com/drive/folders/1Fi4yyKeRFSrix5cQPQOUn3OYy-EWmrn0?usp=sharing>

### Build steps

From the `assignment_3` folder:

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Run the project

Terminal 1:

```bash
. install/setup.bash
ros2 run pf pf_node
```

Terminal 2:

```bash
ros2 bag play data/techboard_log/
```

The executable generates `res.txt`.

To generate the trajectory and timing plot:

```bash
python3 plotter.py
```

## Goals

Localize a forklift using LiDAR and landmarks as reference.

### Main implementation tasks

- Complete the TODOs in `particle_filter.cpp` and `main.cpp` (you may modify any source file).
- Core PF stages to implement/improve: initialization, prediction, update, resampling.
- Random and guess initialization are explicitly evaluated.

### Report

Provide a short report with at least 3 scenarios (1–3 paragraphs per scenario). For each scenario include:

- scenario description and configuration
- estimated forklift trajectory
- execution time discussion

`res.txt` contains:

- best particle estimate (X, Y)
- ground truth (X, Y)
- execution time (from initialization to resampling)

These values are used by `plotter.py`.

## Evaluation metrics (15 points)

- Particle filter works and localizes the vehicle during the whole simulation (7 points)
  - Code quality and optimization are positively evaluated.
  - Submit the source code of your best localization solution.

- Report with scenarios and conclusions (3 points)
  - More scenarios and originality can improve the grade.

- Implement your own resampling method and explain it (2 points)
  - Reference: <https://bisite.usal.es/archivos/resampling_methods_for_particle_filtering_classification_implementation_and_strategies.pdf>

- Implement additional functionality on top of the PF (3+ points)
  - Alternative data association technique
  - PF + Kalman filter hybrid localization
  - Adaptive PF (dynamic particles/resampling frequency based on uncertainty)
  - Any meaningful optimization or extension

## Notes

In this folder you can find reference outputs:

- `pf_slam.txt`: Hipert's best particle filter implementation
- `res.txt`: Nacho's prototype particle filter result
