# pose_estimator_ws

ROS 2 workspace for estimating a drone's pose from its camera images using a neural network. Everything runs in simulation with Gazebo, PX4 and ROS 2.

The workspace contains the `pose_estimator_pkg` package. The `px4_msgs` package is **not** included in this repository: it is downloaded with `vcstool` using the `pose_estimator_ws.repos` file.

## Structure

```
pose_estimator_ws/
├── .gitignore
├── pose_estimator_ws.repos      # external dependencies (px4_msgs)
└── src/
    └── pose_estimator_pkg/
        ├── launch/              # launch files (simple, simulator, lidar, full)
        ├── lidar_config/        # bridge and SLAM configuration
        ├── pose_estimator_pkg/  # Python nodes (+ launch_config.py, shared launch helpers)
        ├── scripts/
        │   └── generate_launch_config.py   # creates launch_config.yaml
        └── launch_config.yaml   # simulation paths and Gazebo world
```

## Prerequisites

### System

- Ubuntu 22.04
- ROS 2 Humble (`desktop` install)
- Gazebo Garden (gz-sim 7)
- PX4-Autopilot v1.14.3 with Gazebo simulation support
- Micro XRCE-DDS Agent (communication between PX4 and ROS 2)
- [pose_estimator_sim](https://github.com/LucyCVieyra/pose_estimator_sim): Gazebo worlds and the custom x500 drone model (cameras + 2D lidar). Follow its README to clone it (default location: `~/pose_estimator_sim`)

### Tools

```bash
sudo apt update
sudo apt install -y git python3-pip python3-colcon-common-extensions \
  python3-rosdep python3-vcstool
```

Initialize `rosdep` if you have never done so:

```bash
sudo rosdep init
rosdep update
```

### Python dependencies

The estimation node uses a neural network, so you need PyTorch and image libraries. Adjust this list to match your environment:

```bash
pip3 install torch torchvision opencv-python numpy pyyaml
```

### GitHub with SSH (only if you clone over SSH)

Your public key must be registered on GitHub (Settings → SSH and GPG keys). Check it with:

```bash
ssh -T git@github.com
```

## Installation

### 1. Clone the workspace

```bash
cd ~
git clone git@github.com:LucyCVieyra/pose_estimator_ws.git
cd pose_estimator_ws
```

Using HTTPS:

```bash
git clone https://github.com/LucyCVieyra/pose_estimator_ws.git
```

### 2. Download `px4_msgs`

```bash
vcs import src < pose_estimator_ws.repos
```

This clones `px4_msgs` into `src/px4_msgs`. The branch must match the PX4 version you use, because the messages change between versions. If you need a different one, edit the `version` field in `pose_estimator_ws.repos` before importing.

### 3. Install ROS dependencies

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build

```bash
colcon build
source install/setup.bash
```

To build only the main package:

```bash
colcon build --packages-select pose_estimator_pkg
```

If you change `px4_msgs`, build it first:

```bash
colcon build --packages-select px4_msgs
colcon build --packages-select pose_estimator_pkg
```

### 5. Source the workspace in every new terminal

```bash
source /opt/ros/humble/setup.bash
source ~/pose_estimator_ws/install/setup.bash
```

To avoid repeating this, add it to `~/.bashrc`.

### 6. Configure the simulation

The launch files read the simulator paths and the Gazebo world from `src/pose_estimator_pkg/launch_config.yaml`. Generate it with the script, which asks for:

- the path of the simulation repository (default `~/pose_estimator_sim`),
- the path of the PX4-Autopilot repository (default `~/PX4-Autopilot`),
- the Gazebo world to load.

```bash
cd ~/pose_estimator_ws/src/pose_estimator_pkg
python3 scripts/generate_launch_config.py
```

```
Simulation repository [~/pose_estimator_sim]:
PX4-Autopilot repository [~/PX4-Autopilot]:

Available Gazebo worlds:
  a) cats_world     Indoor room with obstacles, window tunnels and a landing platform
  b) tugbot_depot   Warehouse (Depot model is downloaded from Gazebo Fuel, needs internet)
Select a world [a]:
```

You can also skip the questions by passing the values as arguments:

```bash
python3 scripts/generate_launch_config.py \
  --sim-repo ~/pose_estimator_sim --px4-repo ~/PX4-Autopilot --world tugbot_depot
```

The resulting file looks like this:

```yaml
sim_environment:
  sim_repo_path: "/home/<user>/pose_estimator_sim"
  px4_repo_path: "/home/<user>/PX4-Autopilot"
  # Gazebo world. Options: cats_world, tugbot_depot
  world: "tugbot_depot"
```

The YAML is installed with the package, so **rebuild after generating or editing it**:

```bash
cd ~/pose_estimator_ws
colcon build --packages-select pose_estimator_pkg
source install/setup.bash
```

### 7. Install the x500 model for the selected world

PX4 always spawns the x500 stored in its own models folder. Each world in `pose_estimator_sim` has its own `x500/model.sdf` (same sensors, different spawn position), so copy the one that matches the selected world. The script and the launch files print this command when the models don't match:

```bash
# cats_world
cp ~/pose_estimator_sim/cats_world/x500/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf

# tugbot_depot
cp ~/pose_estimator_sim/tugbot_depot/x500/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf
```

## Usage

The launch files are in `src/pose_estimator_pkg/launch/`. All of them start the whole simulation stack, so you **don't** need to start the Micro XRCE-DDS Agent, Gazebo or PX4 by hand:

1. Micro XRCE-DDS Agent (`udp4`, port 8888).
2. Gazebo with the world set in `launch_config.yaml`.
3. PX4 SITL (`make px4_sitl gz_x500`). It waits until the Gazebo world is running before starting (up to 120 s), so it attaches to the selected world instead of opening its own empty one.

On top of that, each launch file adds:

| Launch file | Camera bridge | PX4 driver | Lidar + clock bridges | Joystick teleop (`joy_node` + `move_drone`) |
|---|:---:|:---:|:---:|:---:|
| `simple_launch.launch.py` | ✓ | ✓ | | |
| `simulator_launch.launch.py` | ✓ | ✓ | | ✓ |
| `full_launch.launch.py` | ✓ | ✓ | | ✓ |
| `lidar_launch.launch.py` | ✓ | ✓ | ✓ | ✓ |

Example:

```bash
ros2 launch pose_estimator_pkg simple_launch.launch.py
```

The simulation is ready when PX4 prints `Ready for takeoff!`.

### Changing the Gazebo world

1. Run `python3 scripts/generate_launch_config.py` again and pick the other world (or edit `world` in `launch_config.yaml`).
2. Copy the matching x500 model into PX4 (see [step 7](#7-install-the-x500-model-for-the-selected-world)).
3. Rebuild the package: `colcon build --packages-select pose_estimator_pkg`.

## Troubleshooting

- **`Could not read .../launch_config.yaml` or `Missing '...' in launch_config.yaml`**: the config was not generated or the package was not rebuilt after generating it. Run `scripts/generate_launch_config.py` and rebuild.
- **The launch still uses the old world**: the installed copy of `launch_config.yaml` is outdated. Rebuild the package.
- **An empty (flat) Gazebo world opens**: PX4 did not find the selected world and started its own default one. Close any Gazebo instance left over from a previous run and launch again. If the world takes more than 120 s to load (e.g. the first `tugbot_depot` run downloads the Depot model), increase `GZ_WORLD_TIMEOUT_S` in `pose_estimator_pkg/launch_config.py`.
- **`WARNING: the x500 model in PX4 does not match ...`**: copy the x500 model for the selected world (see [step 7](#7-install-the-x500-model-for-the-selected-world)).
- **`Preflight Fail: barometer 0 missing` / `ekf2 missing data` and no `Ready for takeoff!`**: the world SDF is missing Gazebo system plugins required by PX4. A world must load at least the `Physics`, `UserCommands`, `SceneBroadcaster`, `Contact`, `Imu`, `AirPressure` and `Sensors` plugins (use the `gz-sim-*` names) with a `0.004` s physics step, like PX4's `default.sdf`. Make sure your `pose_estimator_sim` copy is up to date.
- **`tugbot_depot` loads with no building**: the Depot model is downloaded from Gazebo Fuel the first time. Check your internet connection and wait for the download.