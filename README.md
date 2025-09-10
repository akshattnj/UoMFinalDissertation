# Autonomous Path Planning for Kinova Gen3 (7-DOF) Using DeepSkin + TRAC-IK

This repository contains the ROS 2 Humble workspace and scripts used in my dissertation project:  
**Autonomous Path Planning on Kinova Gen 3 7-DOF Manipulator for Cleaning Wounds Using Computer Vision**.

## Repository Structure
```
.
├── control_msgs/              # placeholder only
├── gazebo_ros2_control/       # placeholder only
├── gz_ros2_control/           # placeholder only
├── picknik_controllers/       # placeholder only
├── realtime_tools/            # placeholder only
├── ros2_control/              # placeholder only
├── ros2_controllers/          # placeholder only
├── ros2_kortex/               # placeholder only (see Kinova docs)
├── ros2_robotiq_gripper/      # placeholder only
├── ros_gz/                    # placeholder only
├── serial/                    # placeholder only
└── spiral_tracik_executor/    # main package
    ├── scripts/               # segmentation, IK mapping, execution
    ├── images/                # wound samples + masks
    ├── configs/               # DeepSkin + TRAC-IK configs
    ├── results/               # output logs, IoU/seed sensitivity
    └── spiral_executor_node
        └── spiral_executor_node.py  #the code that matters
```

Each placeholder folder has a short README that points to external installation instructions.

---

## Prerequisites

1. **ROS 2 Humble on Ubuntu 22.04**  
   [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. **Kinova ros2_kortex packages**  
   Install following the official documentation:  
   🔗 [Kinovarobotics/ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex/tree/humble#readme-ov-file)

   ```bash
   sudo apt install ros-humble-kortex-bringup \
                    ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config
   ```

3. **MoveIt 2**  
   [Install MoveIt 2 for ROS 2 Humble](https://moveit.ros.org/install-moveit2/binary/)

4. **Python dependencies**
   - `deepskin` (for wound segmentation)
   - `tracikpy` (local build from source)
   - `opencv-python`, `numpy`, `pillow`

---

## Setup

Clone and build this workspace:
```bash
# Create workspace
mkdir -p ~/gen3_ws/src
cd ~/gen3_ws

# Clone this repo
git clone https://github.com/YOUR_USERNAME/gen3_wound_cleaning.git src/gen3_wound_cleaning

# Install Kinova dependencies (not bundled in repo)
vcs import src --skip-existing --input src/gen3_wound_cleaning/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/gen3_wound_cleaning/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/gen3_wound_cleaning/ros2_kortex/simulation.humble.repos

# Install ROS dependencies
rosdep install --ignore-src --from-paths src -y -r

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Usage

### 1. Launch Simulation
```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py
```

### 2. Verify Controllers
```bash
ros2 control list_controllers
```

### 3. Launch RViz + MoveIt
```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config sim.launch.py use_sim_time:=true
```

### 4. Execute Spiral Trajectory
```bash
ros2 run spiral_tracik_executor spiral_executor_node
```

---

## Outputs
- Segmentation masks and trajectory logs are stored under `results/`.  
- Analysis scripts compute precision, IoU, and seed sensitivity statistics.

---

## Citation
If you use this work, please cite:  
> Autonomous Path Planning on Kinova Gen 3 7 DOF Manipulator for Cleaning Wounds Using Computer Vision, MSc Robotics Dissertation, University of Manchester, 2025.
