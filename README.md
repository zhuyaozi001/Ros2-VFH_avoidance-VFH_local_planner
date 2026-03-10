# Ros2-VFH_avoidance-VFH_local_planner
A high-performance obstacle avoidance node using Vector Field Histogram. Features Physical Volume Inflation and Cosine Velocity Scaling to eliminate side-swiping and backward-goal spinning. Optimized for smooth, high-speed navigation in tight spaces.

# About Wpr_simulation2
You can find it at https://github.com/6-robot/wpr_simulation2

# Key Features
-Physical Volume Expansion: Implements robot_radius inflation in polar coordinates (γ=arcsin(R/d)), preventing side-swiping and ensuring the robot clears corners safely.
-Adaptive Velocity Scaling: Uses a Cosine-based scaling law (v=Vmax​⋅cos(Δθ)) to automatically reduce linear speed during sharp turns, eliminating "drifting" and mechanical stress.
-Reactive Goal-Seeking: Dynamically searches the 360° histogram for the safest "valley" closest to the target heading, rather than just choosing the widest opening.
-Zero-Latency Logic: A lightweight, independent node architecture that processes /scan data at sensor-native frequencies for near-instantaneous reaction to moving obstacles.
-Robust Coordinate Alignment: Utilizes angles::shortest_angular_distance to resolve the 0 to 2π wrap-around issue, preventing "infinite spin" or erratic rotation.

# Installation
Prerequisites
-Ubuntu 22.04 + ROS 2 Humble
-equired packages: rclcpp, sensor_msgs, geometry_msgs, nav_msgs, tf2_ros, angles

# build
cd ~/ros2_ws/src
git clone <your-repository-link>
cd ~/ros2_ws
colcon build --packages-select vfh_avoidance
source install/setup.bash

# Quick Start
1.Your project structrue should be like this
vfh_avoidance/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── vfh_node.cpp       # Core VFH logic & ROS 2 wrapper
└── README.md
wpr_simulation2/
...

2.Launch your simulation (e.g., WPR_Simulation2):
ros2 launch wpr_simulation2 world.launch.py

3.Run the VFH Node:
ros2 run vfh_avoidance vfh_node

4.Command the Robot:
Use the 2D Goal Pose tool in RViz2 to set a destination. The robot will autonomously navigate while dodging static and dynamic obstacles.

# Algorithm Overview
The node operates in a three-stage pipeline:
1.Polar Mapping: Raw laser ranges are mapped to N discrete sectors.
2.Obstacle Inflation: Every detected point is expanded into a "sector arc" based on the robot's width, creating a safety buffer.
3.Best Heading Search: The algorithm scans for the sector i that minimizes the angular distance to the goal while maintaining a density H[i]<Threshold.
4.Smooth Control: Linear and angular velocities are coupled to ensure the robot "looks before it leaps," prioritizing orientation before acceleration.

Authors:Gwood - Lead Developer & Algorithm Optimization
