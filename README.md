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
        ├── pose_estimator_pkg/  # Python nodes
        └── scripts/
```

## Prerequisites

### System

- Ubuntu 22.04
- ROS 2 Humble (`desktop` install)
- Gazebo Garden (gz-sim 7)
- PX4-Autopilot with Gazebo simulation support
- Micro XRCE-DDS Agent (communication between PX4 and ROS 2)

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

## Usage

The launch files are in `src/pose_estimator_pkg/launch/`:

| Launch file | Purpose |
|---|---|
| `simple_launch.launch.py` | Basic launch |
| `lidar_launch.launch.py` | Simulator with LiDAR and mapping |
| `full_launch.launch.py` | Full configuration |

Example:

```bash
ros2 launch pose_estimator_pkg full_launch.launch.py
```

Before launching, make sure the PX4 simulator and the Micro XRCE-DDS Agent are running:

```bash
MicroXRCEAgent udp4 -p 8888
```

The launch configuration is in `src/pose_estimator_pkg/launch_config.yaml`.

## Troubleshooting

- 