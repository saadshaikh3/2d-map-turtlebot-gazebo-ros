# 🗺️ 2D Mapping of Robot Environment Using TurtleBot (ROS + Gazebo)

## 🧭 Overview
This project implements a **custom 2D occupancy grid mapping system** using the **TurtleBot3 simulator** in **ROS** with **Gazebo** and **RViz**.  
The system fuses **odometry** and **laser scan** data to generate a real-time 2D map of the robot’s surroundings, enabling autonomous exploration, obstacle avoidance, and dynamic environment visualisation.

The approach uses **trigonometric transformations** to convert local sensor readings into global coordinates and continuously updates the map as the robot moves through the environment.

---

## ⚙️ Core Features
- **Occupancy Grid Mapping:** Real-time construction of a 2D map with cells marked as *free* or *occupied*.  
- **Sensor Fusion:** Combines **laser scan** (`/scan`) and **odometry** (`/odom`) data for accurate spatial estimation.  
- **Autonomous Navigation:** The TurtleBot performs self-driven exploration and obstacle avoidance.  
- **Live Visualisation:** Publishes to `/map` and `/map_metadata` for display in **RViz**.  
- **Real-Time Debugging:** Console output includes robot pose `(x_r, y_r, yaw_r)` for monitoring accuracy.

---

## 🚀 How to Run the Simulation

### 1. Launch the TurtleBot World
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### 2. Open RViz for Visualisation
```bash
rosrun rviz rviz
```

### 3. Run the Custom Mapping Node
```bash
rosrun mapping mapper.py
```
### 4. View the Map

In RViz, add a new display:

Type: Map

Topic: /map

You will now see the robot’s real-time 2D occupancy grid being constructed as it explores.

---

## 📁 File Information
| File	| Description |
|-------|-------------|
|mapper.py |	Python node for fusing odometry and laser scan data to generate the occupancy grid |
|launch/world.launch | Launches the TurtleBot3 world in Gazebo |
|README.md |	Project documentation file |

---

## 🧩 Technical Details

Platform: ROS Noetic

Simulator: Gazebo + RViz

Robot Model: TurtleBot3 (Burger)

Topics Used: /scan, /odom, /map, /map_metadata, /cmd_vel

Language: Python (ROS Node)