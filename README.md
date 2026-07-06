# Autonomous Six-Wheel Rover

A fully autonomous, GPS-denied six-wheel ground rover for buried-threat detection in
hostile terrain. It perceives, localizes and maps (SLAM), plans and drives over rough
ground, runs a suite of buried-threat sensors, makes mission decisions via Behavior
Trees, logs everything for forensic replay, and fails safe.

All intelligence runs on **ROS 2 Humble** (Jetson/Linux); hard-real-time motor control
and a safety watchdog run on a separate **STM32/ODrive** microcontroller over serial/CAN.
The entire stack is developed and validated in the **Gazebo** simulator first — no
hardware required for the current phase.

> See [docs/architecture.md](docs/architecture.md) for the full system design and
> [docs/setup.md](docs/setup.md) for the step-by-step development environment setup.
> No local ROS/Ubuntu install? See [docs/testing.md](docs/testing.md) to build and
> run the whole stack in the cloud (GitHub Actions + Codespaces).

---

## Current status — all 7 phases complete ✅

A full autonomous-rover software stack, developed and runnable entirely in
simulation (no hardware required): a six-wheel rocker-bogie rover in URDF with
virtual LiDAR/IMU/camera, GPS-denied localization + SLAM, terrain-aware Nav2
planning, BehaviorTree mission logic, a buried-threat detection/fusion pipeline,
and a safety/fault system with a software-in-the-loop MCU bridge (plus reference
STM32 firmware).

```
rover_ws/src/
  rover_description/   URDF (xacro), Gazebo sensor/drive plugins, RViz config, display launch
  rover_gazebo/        Test terrain world + full sim launch (Gazebo + spawn + RViz + localization)
  rover_localization/  Dual-EKF (wheel+IMU + SLAM), FAST-LIO2 integration, dead-reckoning manager
  rover_terrain/       Point cloud -> slope/roughness drivability costmap (Nav2-ready)
  rover_navigation/    Nav2 A* global + DWB local over terrain costmap; threat keepout zones
  rover_mission/       BehaviorTree.CPP mission logic (waypoints, stop-scan-mark-reroute, modes)
  rover_detection/     Simulated GPR/metal/VOC, hyperbola detection, per-cell fusion, FAR/m2 eval
  rover_msgs/          Custom interfaces (GprScan, SensorReading, ThreatDetection)
  rover_safety/        Safety/fault manager: watchdog, dropout/stall, e-stop, safe-state command gate
  rover_hw_bridge/     Jetson<->MCU framed link, SITL MCU emulator, reference STM32 firmware
  rover_bringup/       Top-level / teleop / autonomy launch (composition point)
```

## Quick start (after completing docs/setup.md)

```bash
cd ~/P1/rover_ws
colcon build --symlink-install
source install/setup.bash

# Full simulation
ros2 launch rover_gazebo sim.launch.py

# Full simulation + localization (dead-reckoning EKF)
ros2 launch rover_gazebo sim.launch.py localization:=true

# Full autonomy stack (sim + localization + terrain + Nav2)
ros2 launch rover_bringup autonomy.launch.py

# Full autonomy + mission behaviour tree (waypoints, stop-scan-mark-reroute)
ros2 launch rover_bringup autonomy.launch.py mission:=true

# Everything incl. buried-threat detection & fusion pipeline
ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true

# The complete system incl. safety manager + emulated MCU (STM32/ODrive) bridge
ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true hw:=true

# Drive it (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The rover publishes `/points`, `/imu`, `/camera/image_raw`, `/odom`, `/joint_states`
and subscribes to `/cmd_vel`.

## Roadmap (all achievable in sim, no hardware)

1. ✅ Simulation foundation — URDF, Gazebo, RViz.
2. ✅ Localization / SLAM — robot_localization dual-EKF + FAST-LIO2 integration + degraded dead-reckoning.
3. ✅ Terrain assessment — slope/roughness drivability cost field.
4. ✅ Planning — Nav2 global A* + local DWB, threat exclusion polygons.
5. ✅ Mission logic — BT.CPP waypoints, stop-scan-mark-reroute, modes, RTH.
6. ✅ Detection / fusion — GPR + metal/mag/VOC, per-cell threat classifier, FAR/m² evaluation.
7. ✅ Safety + HW bridge — watchdog, fault handling, STM32/ODrive serial/CAN (SITL) + reference firmware.

## Tech stack

- **Middleware:** ROS 2 Humble (DDS, QoS-prioritised topics)
- **Sim:** Gazebo Classic + RViz2 + rosbag2 (replay-driven dev)
- **SLAM/localization:** FAST-LIO2, robot_localization
- **Navigation:** Nav2 (A* global, DWA local)
- **Mission logic:** BehaviorTree.CPP
- **Low-level:** STM32 / ODrive over serial/CAN
- **Compute:** NVIDIA Jetson (Linux)

## License

Apache-2.0
