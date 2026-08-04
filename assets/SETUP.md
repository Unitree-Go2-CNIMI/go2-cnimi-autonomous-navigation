# Setup guide

## Hardware

- Unitree Go2 Edu quadruped robot
- Livox MID-360 LiDAR
- TP Link router
- External PC running the ROS 2 autonomy stack

### 1. Dependencies

Clone and build each of the following before proceeding:

| Package | Repo |
|---|---|
| unitree_ros2 | https://github.com/unitreerobotics/unitree_ros2 |
| Livox-SDK2 | https://github.com/Livox-SDK/Livox-SDK2 |
| livox_ros_driver2 | https://github.com/livox-SDK/livox_ros_driver2 |

Configure the Unitree SDK and Livox driver according to the target hardware setup.

### 2. Network

Ensure the control PC, Unitree Go2, and Livox MID-360 are connected to the same network.

Update the LiDAR host IP in:
```
~/ws_livox/src/livox_ros_driver2/config/MID360_config.json
```


### 3. Clone and Build

Clone this repository:

```bash
git clone https://github.com/Unitree-Go2-CNIMI/go2_cnimi_ws ~/go2_cnimi_ws
```

Install ROS dependencies and build:

```bash
cd ~/go2_cnimi_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 4. Environment Mapping

Navigation requires a pre-built occupancy grid map. A new environment can be mapped using either:

- **SLAM Toolbox** — direct 2D occupancy mapping (run separately, without the main bringup):
```bash
ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py
```
- **FAST-LIO** — 3D LiDAR-inertial mapping, followed by conversion to a navigation-compatible occupancy grid.

The generated map must be available before starting localization and navigation.

### 5. Bring-up

Power on the Go2 and the LiDAR, wait for full boot, then source the workspaces:
```bash
source ~/Livox-SDK2/install/setup.bash
source ~/ws_livox/install/setup.bash
source ~/go2_cnimi_ws/install/setup.bash
source ~/unitree_ros2/setup.sh
```

Launch the stack:
```bash
ros2 launch go2_bringup go2_bringup.launch.py
```

The bring-up brings nodes online in stages. Wait ~90 seconds for everything to settle.

In RViz, set the robot's initial pose with **2D Pose Estimate**; AMCL refines it automatically as scans come in.

### 6. Mission UI

The web interface is maintained in a separate repository: [mission-planner-frontend](https://github.com/Unitree-Go2-CNIMI/mission-planner-frontend).

Clone it then run:

```bash
npm run dev
```


---
