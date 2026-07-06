# Development Environment Setup (Windows -> WSL2 -> ROS 2 Humble + Gazebo)

Everything in this project targets **ROS 2 Humble on Ubuntu 22.04**. On Windows the
recommended path is **WSL2** with an Ubuntu 22.04 distro. This gives you a real Linux
kernel, works with the same commands you'll later run on the Jetson, and supports the
Gazebo/RViz GUIs through WSLg (built into Windows 11 and recent Windows 10).

---

## 1. Install WSL2 + Ubuntu 22.04

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot if prompted. Launch "Ubuntu 22.04" from the Start menu, create your Linux
username/password. Verify you are on WSL **2** and it's Ubuntu 22.04:

```powershell
wsl -l -v          # VERSION column must say 2
```

> GUI apps (Gazebo, RViz) work out of the box via WSLg on Windows 11 / updated Win10.
> Test later with `xeyes` or just by launching Gazebo. Update your GPU driver on the
> Windows side for best OpenGL performance.

---

## 2. Install ROS 2 Humble (inside Ubuntu)

Run these **inside the Ubuntu terminal**:

```bash
# Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable the ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install the desktop bundle (includes RViz2, demos, tutorials)
sudo apt update
sudo apt install -y ros-humble-desktop
```

## 3. Install Gazebo Classic + ROS integration + build tools

```bash
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential

# One-time rosdep init
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 4. Shell setup (source ROS on every new terminal)

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Get the workspace into Linux

The workspace currently lives on the Windows side at `C:\Users\abipathania\Downloads\P1`.
For fast, correct colcon builds, copy `rover_ws` into the **Linux** filesystem (building
inside `/mnt/c/...` is slow and can hit permission issues):

```bash
mkdir -p ~/P1
cp -r "/mnt/c/Users/abipathania/Downloads/P1/rover_ws" ~/P1/
cd ~/P1/rover_ws
```

> Alternatively keep editing on Windows in VS Code and re-copy, or use the VS Code
> **WSL** extension to open `~/P1/rover_ws` directly.

---

## 6. Resolve dependencies and build

```bash
cd ~/P1/rover_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add the workspace to your bashrc for convenience:

```bash
echo 'source ~/P1/rover_ws/install/setup.bash' >> ~/.bashrc
```

---

## 7. Run it

**View the model in RViz only (no physics):**
```bash
ros2 launch rover_description display.launch.py
```

**Full simulation (Gazebo + RViz):**
```bash
ros2 launch rover_gazebo sim.launch.py
```

**Drive it** (separate terminal — this needs keyboard focus):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use `i / j / l / , ` to move, `k` to stop.

**Check the data is flowing:**
```bash
ros2 topic list
ros2 topic hz /points        # LiDAR ~10 Hz
ros2 topic hz /imu           # IMU  ~100 Hz
ros2 topic echo /odom --once # wheel odometry
```

---

## 8. Localization (Phase 2)

The localization stack (`rover_localization`) provides a robot_localization dual-EKF
plus a degraded dead-reckoning manager. The EKF core builds with the standard ROS 2
packages; FAST-LIO2 SLAM is an optional extra component (section 9).

Install robot_localization:
```bash
sudo apt install -y ros-humble-robot-localization
```

**Dead-reckoning only** (wheel + IMU EKF, no SLAM — works immediately):
```bash
ros2 launch rover_gazebo sim.launch.py localization:=true
```
This runs `ekf_local` (publishes `odom -> base_footprint`) and the
`localization_mode_manager`. Inspect it:
```bash
ros2 topic echo /odometry/filtered/local --once
ros2 topic echo /localization/mode          # SLAM_HEALTHY / DEGRADED_DEAD_RECKONING
ros2 run tf2_tools view_frames               # writes frames.pdf of the TF tree
```

**Full SLAM stack** (requires FAST-LIO2 from section 9):
```bash
ros2 launch rover_gazebo sim.launch.py localization:=true slam:=true
```

> Test the degraded mode: while the sim runs, stop the LiDAR (e.g. `ros2 lifecycle`/
> pausing, or temporarily lower its update rate) and watch `/localization/mode` flip to
> `DEGRADED_DEAD_RECKONING` while `ekf_local` keeps publishing `odom -> base_footprint`.

---

## 9. FAST-LIO2 (optional LiDAR-inertial SLAM)

FAST-LIO2 is an external package. Clone it (and its Livox message dependency) into the
workspace `src/` and build:

```bash
cd ~/P1/rover_ws/src

# Livox messages are a build dependency of FAST-LIO2
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# FAST-LIO2 -- the maintained ROS 2 fork (recursive: pulls the ikd-Tree submodule).
# Alternatively: git clone -b ROS2 --recursive https://github.com/hku-mars/FAST_LIO.git
git clone --recursive https://github.com/Ericsii/FAST_LIO.git

sudo apt install -y libpcl-dev libeigen3-dev

cd ~/P1/rover_ws
colcon build --symlink-install
source install/setup.bash
```

> The default branch of `hku-mars/FAST_LIO` is **ROS 1** — you must use `-b ROS2`
> (or the `Ericsii/FAST_LIO` fork, whose default branch is ROS 2).

The sim-tuned parameters live in
`rover_ws/src/rover_localization/config/fastlio_sim.yaml` (in ROS 2
`/**: ros__parameters:` format), and
`rover_localization/launch/fastlio.launch.py` remaps FAST-LIO2's `/Odometry` to
`/fastlio/odometry` for the map-frame EKF.

> **Sim LiDAR caveat:** the Gazebo ray sensor's `PointCloud2` lacks the per-point
> `ring`/`time` fields FAST-LIO2 uses for motion de-skew, so we run with de-skew
> disabled. Expect usable but not hardware-grade odometry in sim; on real
> Velodyne/Ouster/Livox hardware, point to the native driver and re-enable de-skew.

---

## 10. Terrain assessment (Phase 3)

The `rover_terrain` node turns `/points` into a slope/roughness drivability costmap.
It needs a reference-frame TF (`odom` from the wheel plugin or EKF, or `map` with SLAM),
so run it together with the sim:

```bash
# Terrain costmap on top of the dead-reckoning EKF
ros2 launch rover_gazebo sim.launch.py localization:=true terrain:=true
```

Inspect / visualise:
```bash
ros2 topic echo /terrain/costmap --once | head
ros2 run rqt_image_view rqt_image_view   # or view the OccupancyGrid in RViz
```
In RViz the **TerrainCostmap** display (costmap colour scheme) shows free ground as
faint, graded cost as yellow/orange, and lethal slopes/steps (the ramp edges, walls,
rocks) as red. Drive the rover with teleop to accumulate coverage as it explores.

---

## 11. Navigation (Phase 4 — Nav2)

Install Nav2:
```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

Bring up the **full autonomy stack** (sim + localization + terrain + Nav2) in one command:
```bash
ros2 launch rover_bringup autonomy.launch.py
```
This starts A* global planning (NavFn) and DWB local control at 20 Hz, both costmaps
layered on `/terrain/costmap`, plus the threat keepout filter.

**Send a goal:** in RViz click the **Nav2 Goal** tool and click a destination, or:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 6.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```
Watch the green **Plan** path route around the lethal ramp/rocks in the terrain costmap.

**Confirm a threat -> force a reroute:** publish an exclusion polygon; the planner
immediately reroutes around it (this is what the mission layer will call in Phase 5):
```bash
ros2 topic pub --once /threat/add_zone geometry_msgs/msg/PolygonStamped \
  '{header: {frame_id: "odom"}, polygon: {points: [
     {x: 2.5, y: -1.0}, {x: 3.5, y: -1.0}, {x: 3.5, y: 1.0}, {x: 2.5, y: 1.0}]}}'

# Clear all threat zones:
ros2 service call /threat/clear std_srvs/srv/Empty
```

> **SLAM note:** the stack defaults to the `odom` frame (GPS-denied, no map). To plan
> in the drift-corrected `map` frame, run `autonomy.launch.py slam:=true` **and** change
> every `global_frame`/`mask_frame` in `nav2_params.yaml` (and the terrain `map_frame`)
> to `map`.

---

## 12. Mission logic (Phase 5 — BehaviorTree.CPP)

Install BehaviorTree.CPP v3:
```bash
sudo apt install -y ros-humble-behaviortree-cpp-v3
```

Run the full stack **with the mission behaviour tree** driving it:
```bash
ros2 launch rover_bringup autonomy.launch.py mission:=true
```
The mission server ticks the tree in `rover_mission/bt/mission.xml`: it follows the
waypoints in `config/mission.yaml` via Nav2, and reacts each cycle to threats and
blind terrain.

**Exercise the stop-scan-mark-reroute cycle** — report a detected threat (this is what
the Phase 6 detection layer will publish); the rover stops, marks a keepout zone, and
Nav2 reroutes:
```bash
ros2 topic pub --once /detection/threat geometry_msgs/msg/PointStamped \
  '{header: {frame_id: "odom"}, point: {x: 4.0, y: -1.0, z: 0.0}}'
```

**Switch operational mode** (Standard | Autonomous | Ghost | EmergencyReturn | LostCommsAutonomous):
```bash
# Abort and return home:
ros2 topic pub --once /mission/mode std_msgs/msg/String '{data: "EmergencyReturn"}'
# Zero-RF (suppresses /mission/status telemetry):
ros2 topic pub --once /mission/mode std_msgs/msg/String '{data: "Ghost"}'
```

**Simulate blind terrain** (drives the drone-launch trigger): the mission watches
`/localization/mode`; when it reads `DEGRADED_DEAD_RECKONING` the tree fires
`/drone/launch` once.

---

## 13. Detection & fusion (Phase 6)

No hardware sensors exist in Gazebo, so this layer simulates the buried-threat
sensor suite from a ground-truth map and runs the real processing/fusion pipeline
on top. It needs only the workspace packages (numpy), plus `rover_msgs` built.

Run the **complete pipeline** (sim + localization + terrain + Nav2 + mission + detection):
```bash
ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true
```
As the rover follows its waypoints it drives over the ground-truth threats defined
in `rover_detection/config/detection.yaml`. The pipeline:
1. `sensor_sim_node` emits GPR A-scans (`/gpr/scan`) + metal/VOC readings.
2. `gpr_processor_node` does background subtraction + hyperbola detection.
3. `fusion_node` fuses per cell -> `/detection/confidence` (RViz) and, once a cell
   is confirmed, publishes `/detection/threat` -> the mission layer stops, marks a
   keepout zone, and Nav2 reroutes. Confirmations are logged to `~/rover_detections.jsonl`.
4. `detection_eval_node` scores `Pd` and **false-alarm-rate per m²** vs ground truth.

Watch it work:
```bash
ros2 topic echo /detection/threats           # classified detections
ros2 topic echo /detection/metrics           # Pd / FAR-per-m2
cat ~/rover_detections.jsonl                  # forensic replay log
```
In RViz the **ThreatConfidence** layer shows the fused confidence field building up
over the threats. Tune the false-alarm-rate via `confirm_confidence`,
`min_observations`, and the fusion weights in `detection.yaml`.

---

## 14. Safety & hardware bridge (Phase 7)

This layer adds the supervisory safety manager plus a software-in-the-loop (SITL)
model of the microcontroller that runs the hard-real-time motor loops and
watchdog. No extra apt packages are needed.

Run the **complete system** (everything, including safety + the emulated MCU):
```bash
ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true hw:=true
```
Command flow becomes: Nav2/mission `-> /cmd_vel -> safety_manager -> /cmd_vel_safe
-> hw_bridge -> (framed link) -> mcu_sim -> /motor/cmd_vel -> Gazebo`.

Watch safety in action:
```bash
ros2 topic echo /safety/state       # NOMINAL | DEGRADED | SAFE_STOP | ESTOP
ros2 topic echo /mcu/fault          # MCU fault bitmask (watchdog/overcurrent/stall)
```

**Trigger an emergency stop** (latched safe state; motors go to zero):
```bash
ros2 topic pub --once /safety/estop std_msgs/msg/Bool '{data: true}'
# clear it:
ros2 service call /safety/reset std_srvs/srv/Trigger
```

**Test the MCU watchdog** (the hard-real-time last line of defence): kill the
`safety_manager` or `hw_bridge` node — with no fresh command frames, the emulated
MCU's watchdog fires within ~200 ms and forces the motors to zero (`FAULT_WATCHDOG`),
exactly as the STM32 firmware in `rover_hw_bridge/firmware/stm32/control_loop.c` does.

The reference firmware for a real STM32 (or an ODrive) lives in
[rover_hw_bridge/firmware](../rover_ws/src/rover_hw_bridge/firmware); the SITL
`mcu_sim_node` mirrors it so hardware can be swapped in later without touching the
rest of the stack.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Gazebo window is black / crashes on start | Update Windows GPU driver; try `export LIBGL_ALWAYS_SOFTWARE=1` for software rendering |
| `spawn_entity.py` times out | Give Gazebo a few seconds; ensure `/robot_description` is published (`ros2 topic echo /robot_description --once`) |
| No `/points` topic | The CPU `ray` sensor is heavy; confirm Gazebo isn't paused; check `ros2 node list` for the sensor |
| `xacro: command not found` | `sudo apt install ros-humble-xacro` and re-source |
| Slow build on `/mnt/c` | Copy the workspace into the Linux home directory (step 5) |

---

## Why WSL2 and not native Windows?

ROS 2 Humble's binary packages, Gazebo Classic, FAST-LIO2, Nav2, and BT.CPP are all
built and tested for Ubuntu 22.04. Developing on the same OS as the Jetson target means
the code you write in sim is the code that ships — no cross-platform surprises.
