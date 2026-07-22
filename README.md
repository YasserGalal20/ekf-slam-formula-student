# EKF-SLAM for a Formula Student Driverless Race Car

An Extended Kalman Filter that estimates a race car's pose **and** builds a map of the track cones at the same time, on a track it has never seen before. Written in Python for ROS, developed and tested in Gazebo.

Alongside the code there is a **full tutorial** ([`docs/`](docs/)) explaining the whole thing from first principles — averages and variance, through Gaussians, the Kalman filter, Taylor series and Jacobians, to the implementation itself — assuming no background in robotics or statistics.

---

## Contents

| Path | What it is |
|---|---|
| [`src/final_ekf.py`](src/final_ekf.py) | The EKF-SLAM ROS node. |
| [`docs/`](docs/) | The full write-up, in `.docx` and `.pdf`. |

---

## The problem

A Formula Student driverless car has to race around a track marked out by traffic cones, with nobody in it, having never seen the track before. Every instant it needs to answer two questions at once:

1. **Where am I?**
2. **Where are the cones?**

Answering both simultaneously is the SLAM problem. Dead reckoning alone drifts by metres within a minute, and on a track three metres wide, metres of drift means hitting cones.

## The approach

An Extended Kalman Filter holding the car's pose *and* every discovered cone in a single state vector:

```
x = [ x_car, y_car, yaw | cone_1x, cone_1y | cone_2x, cone_2y | ... ]
```

The state grows as cones are discovered. Because the covariance matrix tracks how the car's error correlates with each cone's error, re-observing a known cone corrects the pose *and* the rest of the map — which is what makes loop closure work.

Each cycle:

- **Predict** — move the estimate forward with a kinematic motion model. Uncertainty grows.
- **Update** — for each detected cone: associate it with a known cone (or add it), compute the innovation, compute the Kalman gain, correct. Uncertainty shrinks.

**Why EKF and not GraphSLAM?** Compute budget. Le Large's KIT thesis benchmarked both on real Formula Student data against DGPS ground truth and found GraphSLAM more accurate but far heavier on CPU, concluding EKF-SLAM remains "a valid option" where resources are constrained. That was our situation. Discussed in §21.4 of the docs.

## Key design decisions

- **Range–bearing measurements**, not Cartesian — matches what a lidar physically measures, so the noise model is honest.
- **Mahalanobis distance for data association**, so a discrepancy is judged in standard deviations rather than metres, and one threshold stays meaningful even though it mixes metres with radians.
- **6 m detection cutoff** — beyond that cone detections became unreliable, and a bad measurement is worse than no measurement.
- **Sequential per-cone updates** rather than one batch update — same result for independent measurements, far simpler, and avoids inverting a large matrix.

---

## Running it

Requires ROS 1 (Melodic/Noetic), `rospy`, `tf`, `numpy`, and the team's `aam_common_msgs`.

```bash
roslaunch aamfsd_gazebo <your_track>.launch   # simulator + perception
rosrun <your_package> final_ekf.py            # this filter
rviz                                          # fixed frame: map
```

In RViz add `/yassers_path` (trajectory) and `/global_map_yasser` (cone map).

### Topics

**Subscribes**

| Topic | Type | Carries |
|---|---|---|
| `/centroid_lidar_cones_marker` | `MarkerArray` | Detected cones, car frame |
| `/vel_ekf` | `Float32MultiArray` | `Vx`, `Vy`, heading, yaw |
| `/sensor_imu_hector` | `Imu` | Angular velocity |
| `/robot_control/command` | `AckermannDriveStamped` | Commanded speed and steering |

**Publishes**

| Topic | Type | Carries |
|---|---|---|
| `/yassers_path` | `Path` | Estimated trajectory |
| `/global_map_yasser` | `MarkerArray` | Estimated cone map |

### Tuning

| Constant | Value | Meaning |
|---|---|---|
| `Cx` | `diag(0.25, 0.25, 10°)²` | Process noise — how much to distrust the motion model |
| `R` | `diag(0.25, 5°)²` | Measurement noise — how much to distrust the lidar |
| `MAX_RANGE` | 6.0 m | Ignore cones beyond this |
| `M_DIST_TH` | 1.0 | Association threshold (Mahalanobis, dimensionless) |

Only the **ratio** of process to measurement noise affects the estimate — scaling both changes nothing. Measure `R` from stationary data, then tune `Cx` against it. See §21 of the docs.

---

## References

Full list in the document.

- Le Large, N. (2020). *A comparison of different approaches to solve the SLAM problem on a Formula Student Driverless race car.* MSc thesis, Karlsruhe Institute of Technology.
- Thrun, Burgard & Fox (2005). *Probabilistic Robotics.* MIT Press.
- Sakai, A. et al. *[PythonRobotics](https://atsushisakai.github.io/PythonRobotics/)* — the EKF-SLAM implementation here follows its structure and naming.
- Becker, A. *[kalmanfilter.net](https://www.kalmanfilter.net/)*
- Kálmán, R. E. (1960). "A New Approach to Linear Filtering and Prediction Problems."

## Licence

MIT — see [LICENSE](LICENSE).

## Author

**Yasser Galal** — Formula Student, Autonomous Systems.
