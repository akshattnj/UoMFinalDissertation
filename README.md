# Autonomous Path Planning for Kinova Gen3 (7-DOF) Using DeepSkin + TRAC-IK

This repository contains the ROS 2 Humble workspace and scripts used in my dissertation project:  
**Autonomous Path Planning on Kinova Gen 3 7-DOF Manipulator for Cleaning Wounds Using Computer Vision**.

## Repository Structure (Full Workspace)
When fully set up with dependencies, the workspace should look like this:
```
.
├── control_msgs
├── gazebo_ros2_control
├── gz_ros2_control
├── picknik_controllers
├── realtime_tools
├── ros2_control
├── ros2_controllers
├── ros2_kortex              # cloned separately inside this workspace
├── ros2_robotiq_gripper
├── ros_gz
├── serial
├── tracikpy                 # included here for pip install
└── spiral_tracik_executor    # main package from this repository
```

**Note:** Only `spiral_tracik_executor/` and a local `tracikpy/` folder (for pip installation) are included in this repository.  
All other packages (including `ros2_kortex`, `ros2_control`, etc.) must be cloned and imported inside this workspace.

---

## Prerequisites

1. **ROS 2 Humble on Ubuntu 22.04**  
   [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. **Kinova ros2_kortex packages**  
   Clone directly inside this repository:
   ```bash
   cd src/UoMFinalDissertation
   git clone -b humble --single-branch https://github.com/Kinovarobotics/ros2_kortex.git ros2_kortex
   ```
   Then pull dependencies:
   ```bash
   vcs import src --skip-existing --input src/UoMFinalDissertation/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
   vcs import src --skip-existing --input src/UoMFinalDissertation/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
   vcs import src --skip-existing --input src/UoMFinalDissertation/ros2_kortex/simulation.humble.repos
   ```

3. **Python dependencies**
   - `deepskin` (for wound segmentation)
   - `tracikpy` (installed locally from the `tracikpy/` folder in this repo)
   - `opencv-python`, `numpy`, `pillow`

---

## Setup

Clone and build this workspace:
```bash
# Create workspace
mkdir -p ~/gen3_ws/src
cd ~/gen3_ws

# Clone this repository
git clone https://github.com/akshattnj/UoMFinalDissertation.git src/UoMFinalDissertation

# Clone ros2_kortex inside the repository
cd src/UoMFinalDissertation
git clone -b humble --single-branch https://github.com/Kinovarobotics/ros2_kortex.git ros2_kortex

# Pull additional dependencies
cd ~/gen3_ws
vcs import src --skip-existing --input src/UoMFinalDissertation/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/UoMFinalDissertation/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/UoMFinalDissertation/ros2_kortex/simulation.humble.repos

# Install ROS dependencies
rosdep install --ignore-src --from-paths src -y -r

# Install tracikpy from the local folder
cd src/UoMFinalDissertation/tracikpy
pip install .

# Build the workspace
cd ~/gen3_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Usage

---
### 1. Launch Simulation
To launch the Kinova Gen3 7-DOF in Gazebo with RViz:
```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
sim_gazebo:=true \
use_sim_time:=true \
robot_type:=gen3 \
dof:=7 \
gripper:=robotiq_2f_85 \
robot_name:=gen3 \
robot_controller:=joint_trajectory_controller \
launch_rviz:=true
```

### 2. Verify Controllers
```bash
ros2 control list_controllers
```

### 3. Execute Spiral Trajectory
```bash
ros2 run spiral_tracik_executor spiral_executor_node
```

## Outputs
- Segmentation masks and trajectory logs are stored under `results/`.  
- Analysis scripts compute precision, IoU, and seed sensitivity statistics.

---

## License Notice for tracikpy
The `tracikpy` folder included in this repository is distributed under the MIT License, originally authored by **Michael Danielczuk (c) 2021**.
It has been added here to allow local installation via `pip install .`.


For details, please see the `LICENSE` file inside the `tracikpy/` folder.


---

## Citation
If you use this work, please cite:  
> Autonomous Path Planning on Kinova Gen 3 7 DOF Manipulator for Cleaning Wounds Using Computer Vision, MSc Robotics Dissertation, University of Manchester, 2025.
