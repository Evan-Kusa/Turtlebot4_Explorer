# TurtleBot 4 Autonomous Explorer

This project implements an autonomous exploration system for the TurtleBot 4, using ROS 2 for navigation and SLAM. The code enables the robot to explore unknown environments by detecting frontiers in a map, selecting the most promising frontier to explore, and navigating towards it. The goal is to cover as much of the environment as possible.

## Features

- **Autonomous Exploration**: The robot identifies frontiers (boundaries between known and unknown areas) and navigates to the nearest one to explore new spaces.
- **Frontier Detection**: Detects frontier cells in the occupancy grid (free space adjacent to unknown space).
- **Frontier Grouping**: Groups frontier cells and selects the largest ones for exploration.
- **Navigation**: Uses the TurtleBot 4 Navigator to move the robot to goal positions based on frontier detection.
- **Real-Time Map Updates**: Subscribes to `map` and `odom` topics to receive real-time updates of the environment and the robot’s position.

## ROS 2 Topics and Messages

- **Subscribed Topics**:
  - `/map` (`nav_msgs/OccupancyGrid`): Receives the occupancy grid map of the environment.
  - `/odom` (`nav_msgs/Odometry`): Receives the robot’s odometry data for position and orientation.

- **Published Topics**:
  - `/cmd_vel` (`geometry_msgs/Twist`): Sends velocity commands to the robot for movement.

## Code Overview

### Main Components

- **AutonomousExplorer**: The main ROS 2 node that orchestrates the robot’s exploration. It subscribes to map and odometry topics, detects frontiers, and commands the robot to explore them.
  
- **Frontier Detection**: The function `detect_frontiers()` identifies frontiers in the occupancy grid by checking for unknown space adjacent to free space.

- **Frontier Grouping**: Frontier cells are grouped using depth-first search (DFS) to form connected clusters. These groups are then filtered to select the largest ones for exploration.

- **Navigation**: The best frontier group is selected, and the robot is commanded to navigate to its centroid using the TurtleBot 4 Navigator.

### Parameters

The parameters for exploration and navigation can be adjusted in the `load_parameters()` function. The key parameters include:

- `lookahead_distance`: The distance ahead of the robot to consider when planning.
- `speed`: Maximum speed of the robot.
- `expansion_size`: Factor used to expand walls in the occupancy grid for safety.
- `target_error`: Allowable distance error when reaching a goal.
- `robot_radius`: Radius of the robot for safety checks.
- `tick_rate`: The rate at which the exploration loop runs.

### Key Functions

- `detect_frontiers(occupancy_grid)`: Detects frontier cells in the map.
- `assign_frontier_groups(frontier_grid)`: Groups frontier cells into clusters using DFS.
- `filter_frontier_groups(groups)`: Filters and selects the largest frontier groups for exploration.
- `find_best_frontier_group(groups, current_position)`: Selects the best frontier group to navigate to, based on proximity to the robot.

### Launch Instructions

1. **Prerequisites**:
   - ROS 2 Humble or later.
   - TurtleBot 4.
   - SLAM package installed and running.

2. **Launching the Code**:
   To launch the autonomous explorer, follow these steps:

   1. Install ROS 2 and TurtleBot 4 Navigation packages.
   2. Run SLAM to generate a map of the environment.
   3. Execute the script:
      ```bash
      python3 autonomous_explorer.py
      ```

## Dependencies

- ROS 2 (Humble or later)
- `nav_msgs` for OccupancyGrid and Odometry
- `geometry_msgs` for PoseStamped and Twist
- `turtlebot4_navigation` for TurtleBot4Navigator

## Notes

- This system assumes the TurtleBot 4 is undocked and ready for exploration. If the robot is docked, it will automatically undock and begin exploring.
- Ensure the environment is set up with SLAM running for map generation.

## Future Improvements

- Add support for more complex path planning algorithms.
- Improve the handling of dynamic obstacles.
- Implement multi-robot exploration for larger environments.

## License

This project is licensed under the BSD-3-Clause License.

## Contact

For questions or issues, please contact the author.

