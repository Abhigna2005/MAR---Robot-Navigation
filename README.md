# Autonomous Mobile Robot Navigation (ROS 2)

## Overview
This project implements a fully autonomous mobile robot navigation system using ROS 2 Humble, Gazebo, and RViz2.

A differential-drive robot is spawned in a simulated parking lot and autonomously navigates to a goal parking slot using:
- A* path planning
- Odometry-based control
- LiDAR-based obstacle detection
- Real-time visualization in RViz

---

## Objectives
- Navigate from start position (0.0, 0.5) to goal (1.5, -4.0)
- Compute shortest collision-free path using A*
- Enforce speed constraints:
  - Max linear velocity: 0.7 m/s
  - Max angular velocity: 1.2 rad/s
- Reduce speed during turns and near obstacles
- Visualize robot, path, and goal in RViz
- Generate navigation performance metrics

---

## Technologies Used
- ROS 2 Humble
- Gazebo Simulator
- RViz2 Visualization
- Python 3
- URDF/Xacro (Robot Modeling)

---


## ROS Topics Used

| Topic | Description |
|------|------------|
| /cmd_vel | Velocity commands |
| /odom | Robot position (odometry) |
| /scan | LiDAR sensor data |
| /planned_path | Planned path |
| /trajectory | Actual path followed |
| /goal_marker | Goal visualization |

---

## Robot Description
- Differential drive robot
- Two powered wheels + passive caster
- 2D LiDAR sensor (360° scan)
- URDF/Xacro-based modular design

---

## Simulation Environment
- 20m × 16m parking lot
- Boundary walls
- Parked cars
- Speed bumps
- Bollards
- Empty parking slot as goal

---

## Path Planning (A*)
- Grid size: 80 × 64
- Resolution: 0.25 m per cell
- Obstacle inflation: 0.5 m
- 8-directional movement
- Path smoothing applied

---

## Navigation & Control
- Waypoint-following controller
- Uses /odom feedback
- Speed control:
  - Slows during turns
  - Smooth acceleration and deceleration
- LiDAR-based obstacle avoidance

---

## RViz Visualization
Displays:
- Planned path (blue)
- Robot trajectory (orange)
- Goal marker (green)
- LiDAR scan (red)
- Robot model

---

## Evaluation Metrics
- Path length (planned vs actual)
- Travel time
- Path optimality ratio
- Navigation accuracy
- Speed compliance

---

## How to Run

### Option 1: Single Launch
source /opt/ros/humble/setup.bash  
ros2 launch launch/robot_launch.py  

---

### Option 2: Manual Execution

Terminal 1 (Gazebo):
gazebo worlds/parking_lot.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so  

Terminal 2 (Robot State Publisher):
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro robot/robot.urdf.xacro)"  

Terminal 3 (Spawn Robot):
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot -x 0.0 -y 0.5 -z 0.1  

Terminal 4 (RViz):
rviz2 -d rviz/navigation.rviz  

Terminal 5 (Planner):
python3 navigation/astar_planner.py  

---

## Key Concepts
- A* Path Planning
- Occupancy Grid Mapping
- Differential Drive Kinematics
- Odometry-Based Navigation
- LiDAR-based Obstacle Detection

---

## Future Improvements
- SLAM-based mapping
- Dynamic obstacle handling
- Real robot implementation
- Advanced planners (DWA, TEB)

---

## Conclusion
This project demonstrates a complete robotics pipeline integrating simulation, planning, control, and visualization using ROS 2. The robot successfully performs autonomous navigation with high accuracy and efficiency.
