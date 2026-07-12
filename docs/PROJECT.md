# Year 3 Project — ROS 2 SLAM on Jetson Orin (EEE3017)

Personal portfolio mirror of the University of Surrey **EEE3017 Year 3 Project** deployment stack.

**Companion visualisation repo:** [ROS_Robot_Vis](https://github.com/maclajurekifl/ROS_Robot_Vis)

## What this is

ROS 2 (**Humble**) workspace targeting **NVIDIA Jetson Orin** with:

- Livox MID360 LiDAR driver
- Planar EKF localisation
- Optional **PCL NDT** or **FAST-LIO** LiDAR odometry
- Keyframe map + pose graph

## Documentation

- `docs/EEE3017_Dissertation.pdf` — final dissertation submission
- `src/LiDAR-Instructions.md` — LiDAR / stack notes
- `CHANGES-CONFIGS` — configuration change log

## Related repositories

| Repo | Role |
|------|------|
| **This repo (`ROS_Deploy_Orin`)** | On-robot SLAM / sensor stack |
| [`ROS_Robot_Vis`](https://github.com/maclajurekifl/ROS_Robot_Vis) | Minimal RViz / URDF visualisation |

## Author

Jude Burton — University of Surrey, Electrical & Electronic Engineering
