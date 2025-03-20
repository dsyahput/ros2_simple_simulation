# ros2_simple_simulation

## Overview

This repository contains a simple ROS 2 simulation setup for testing with **ROS 2 Humble**, **Gazebo Ignition Fortress**, and **RViz**.

## Tested On

- **ROS 2 Humble**
- **Gazebo Ignition Fortress**
- **RViz2**

The robot meshes file is obtained from this repository: [razbot_tutorials](https://github.com/Waterfox/razbot_tutorials.git)

## Setup Instructions

### Clone the Repository
```bash
git clone https://github.com/dsyahput/ros2_simple_simulation.git
cd ros2_simple_simulation
```

### Build the Workspace
```bash
colcon build
source install/setup.bash
```

### Run the Simulation

1. Launch the simulation environment:
   ```bash
   ros2 launch rosbot_description rosbot_simulation.launch.py
   ```

2. Spawn the controllers:
   ```bash
   ros2 launch rosbot_control spawn_controllers.launch.py
   ```

3. Run the command velocity publisher:
   ```bash
   ros2 run cmd_vel_publisher_cpp cmd_vel_publisher
   ```

4. Visualize in RViz2:
   ```bash
   ros2 launch rosbot_description rviz_visualize.launch.py
   ```