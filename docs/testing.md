# Testing without a local ROS / Ubuntu install

Everything here runs **in the cloud** — nothing is installed on your (locked-down)
laptop. You only need a browser, plus a free GitHub account. The simulation runs
**headless** (the LiDAR is CPU-based, no GPU required), so it works on cloud
runners and small cloud VMs.

## Prerequisite: put the project on GitHub

The cloud options read the code from a GitHub repository.

```powershell
cd C:\Users\abipathania\Downloads\P1
git init
git add .
git commit -m "Autonomous six-wheel rover stack"
# create an EMPTY private repo on github.com, then:
git remote add origin https://github.com/<you>/<repo>.git
git branch -M main
git push -u origin main
```

The [.gitignore](../.gitignore) already excludes `build/`, `install/`, `log/`, and
rosbags.

---

## Option A — GitHub Actions CI (automated, zero setup, no GUI)

Already configured in [.github/workflows/ci.yml](../.github/workflows/ci.yml). The
moment you push, GitHub spins up an Ubuntu 22.04 + ROS 2 Humble container and:

1. Resolves all dependencies with `rosdep` (Nav2, robot_localization, BT.CPP, …).
2. **Builds all 11 packages** (C++ `rover_mission`, the `rover_msgs` interfaces,
   and every Python package).
3. Runs `colcon test`.
4. Parses the URDF with `xacro` in both the bare and hardware-bridge configs.
5. **Runs a headless Gazebo smoke test** — launches sim + localization + terrain
   and asserts that `/points`, `/imu`, `/odom`, `/joint_states` are publishing.

Watch results under the repo's **Actions** tab. This catches the large majority of
issues (compile errors, missing deps, launch wiring, plugin loading, node startup,
URDF validity) with **no interaction and no GUI**. It's free for public repos and
has a generous free tier for private ones.

> This is the fastest way to know the whole stack builds and the sim comes up.

---

## Option B — GitHub Codespaces (full ROS 2 in a browser)

For **interactive** testing (drive the rover, send Nav2 goals, watch the threat
pipeline), open the repo in a Codespace. It uses
[.devcontainer/devcontainer.json](../.devcontainer/devcontainer.json), which is a
ROS 2 Humble container that auto-builds the workspace.

1. On the GitHub repo page: **Code ▸ Codespaces ▸ Create codespace on main**.
2. Wait for `postCreate` to finish (it installs deps and runs `colcon build`).
3. Open a terminal in the browser VS Code and run the stack **headless**:
   ```bash
   ros2 launch rover_gazebo sim.launch.py headless:=true rviz:=false \
     localization:=true terrain:=true
   ```

**Visualise it** (two choices):

- **Foxglove (recommended, lightweight):** in a second terminal run
  ```bash
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml
  ```
  Forward port **8765**, then open <https://app.foxglove.dev>, choose
  "Open connection ▸ Foxglove WebSocket", and point it at the forwarded URL. You
  can view the robot model, `/points`, `/tf`, the camera, `/terrain/costmap`,
  `/global_costmap/costmap`, the planned `/plan`, and `/detection/confidence`.
  This shows everything the robot perceives and decides — the Gazebo *world*
  window isn't needed.

- **Full Gazebo/RViz GUI (noVNC):** the devcontainer includes a lightweight web
  desktop. Forward port **6080**, open it in a browser (password `rover`), then in
  a container terminal run without `headless:=true`:
  ```bash
  ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true
  ```
  The Gazebo and RViz windows appear inside the browser desktop. (Heavier than
  Foxglove; use a 4-core Codespace.)

Run the full autonomy + detection + safety stack the same way you would locally:
```bash
ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true hw:=true
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 6.0, y: 0.0}, orientation: {w: 1.0}}}'
```

> Codespaces free tier includes a monthly quota of core-hours + storage — plenty
> for iterative testing. Stop the Codespace when idle to preserve quota.

---

## Option C — The Construct (browser ROS + Gazebo, easiest GUI)

<https://www.theconstruct.ai> is a purpose-built browser ROS environment (ROS 2
Humble, Gazebo, RViz, all in the browser, nothing installed). Good if Codespaces
GUI is fiddly:

1. Create a free account and start a **ROS 2 Humble** rosject.
2. Clone this repo into `~/ros2_ws/src` (or upload it), then
   `cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build`.
3. Use the built-in Gazebo and RViz windows (the "Graphical Tools" / "Gazebo"
   tabs) to run and watch the sim.

---

## Option D — Docker Desktop (only if IT allows it locally)

If your laptop permits Docker Desktop (it needs the WSL2 or Hyper-V backend, which
may be blocked), you can run the same container locally:
```powershell
docker run -it --rm -v ${PWD}:/work -w /work/rover_ws osrf/ros:humble-desktop-full bash
# inside: rosdep install --from-paths src --ignore-src -r -y && colcon build
```
GUI apps then need an X server (e.g. VcXsrv). Usually Codespaces/CI are simpler on
a locked-down machine.

---

## What each option proves

| | Builds C++/msgs | Launch wiring | Headless sim | Interactive drive | 3D GUI |
|---|:--:|:--:|:--:|:--:|:--:|
| A. GitHub Actions | ✅ | ✅ | ✅ | — | — |
| B. Codespaces + Foxglove | ✅ | ✅ | ✅ | ✅ | perception only |
| B. Codespaces + noVNC | ✅ | ✅ | ✅ | ✅ | ✅ |
| C. The Construct | ✅ | ✅ | ✅ | ✅ | ✅ |

Start with **Option A** (push and watch the Actions tab) to confirm the whole
stack builds and boots, then use **Option B** when you want to fly the rover.

## FAST-LIO2 note

The optional SLAM backend (`slam:=true`) is an external package that must be
cloned/built (see [setup.md](setup.md) §9). CI and the default cloud runs use the
GPS-denied `odom`-frame configuration and do **not** require it.
