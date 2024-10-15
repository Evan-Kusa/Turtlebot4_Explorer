# Copyright 2024 Evan Kusa
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of Evan Kusa nor the names of its contributors may be used
#    to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import threading
import time
from rclpy.executors import SingleThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import Twist

# Hardcoded parameters
def load_parameters():
    return {
        "lookahead_distance": 0.24,  # forward look-ahead distance
        "speed": 0.5,               # maximum speed
        "expansion_size": 3,         # wall expansion factor
        "target_error": 0.15,        # allowable error to target
        "robot_radius": 0.2,          # robot radius for local safety
        "tick_rate": 0.5,             # Exploration loop tick rate in seconds.
    }

params = load_parameters()

TARGET_ERROR = params.get("target_error", 0.2)
ROBOT_RADIUS = params.get("robot_radius", 0.2)
TICK_RATE = params.get("tick_rate", 0.5)
SPEED = params.get("speed", 0.5)


global_goal = None

def euler_from_quaternion(x, y, z, w):
    """
    Converts quaternion (x, y, z, w) to Euler angles (yaw).
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z


def detect_frontiers(occupancy_grid):
    """
    Detects frontier cells in the occupancy grid.
    """
    frontier_grid = np.zeros_like(occupancy_grid)
    rows, cols = occupancy_grid.shape
    for i in range(rows):
        for j in range(cols):
            if occupancy_grid[i][j] == 0:  # Free space
                # Check adjacent cells for unknown space (-1)
                if (i > 0 and occupancy_grid[i - 1][j] == -1) or \
                   (i < rows - 1 and occupancy_grid[i + 1][j] == -1) or \
                   (j > 0 and occupancy_grid[i][j - 1] == -1) or \
                   (j < cols - 1 and occupancy_grid[i][j + 1] == -1):
                    frontier_grid[i][j] = 2  # Mark as frontier
    return frontier_grid

def depth_first_search_iterative(grid, i, j, group_id, groups):
    """
    Performs iterative DFS to group connected frontier cells.
    """
    stack = [(i, j)]
    rows, cols = grid.shape

    while stack:
        ci, cj = stack.pop()

        # Check bounds
        if ci < 0 or ci >= rows or cj < 0 or cj >= cols:
            continue

        # If the cell is not a frontier, skip
        if grid[ci][cj] != 2:
            continue

        # Mark as visited
        grid[ci][cj] = 0
        groups.setdefault(group_id, []).append((ci, cj))

        # Add neighboring cells to stack
        for dx, dy in [(-1, -1), (-1, 0), (-1, 1),
                       (0, -1),         (0, 1),
                       (1, -1),  (1, 0),  (1, 1)]:
            ni, nj = ci + dx, cj + dy
            stack.append((ni, nj))

def assign_frontier_groups(frontier_grid):
    """
    Assigns group IDs to frontier cells.
    """
    group_id = 1
    groups = {}
    rows, cols = frontier_grid.shape
    for i in range(rows):
        for j in range(cols):
            if frontier_grid[i][j] == 2:
                depth_first_search_iterative(frontier_grid, i, j, group_id, groups)
                group_id += 1
    return frontier_grid, groups

def filter_frontier_groups(groups, min_size=5, max_groups=5):
    """
    Filters frontier groups based on size and selects the largest ones.
    """
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    filtered_groups = [group for group in sorted_groups if len(group[1]) >= min_size]
    return filtered_groups[:max_groups]

def calculate_centroid(cells):
    """
    Calculates the centroid of a group of cells.
    """
    x_coords = [cell[0] for cell in cells]
    y_coords = [cell[1] for cell in cells]
    n = len(cells)
    if n == 0:
        return None
    mean_x = sum(x_coords) / n
    mean_y = sum(y_coords) / n
    return int(mean_x), int(mean_y)

def find_best_frontier_group(groups, current_position):
    """
    Finds the best frontier group to explore next based on proximity.
    """
    best_distance = float('inf')
    best_centroid = None

    for group_id, cells in groups:
        # Calculate centroid of the group
        centroid = calculate_centroid(cells)
        if centroid is None:
            continue
        # Calculate Euclidean distance
        distance = math.hypot(current_position[0] - centroid[0],
                              current_position[1] - centroid[1])
        if distance < best_distance:
            best_distance = distance
            best_centroid = centroid

    return best_centroid

def exploration(occupancy_grid, resolution, origin_x, origin_y, current_position):
    """
    Main exploration function that updates the global goal.
    """
    global global_goal
    grid = occupancy_grid.copy()

    # Detect frontier cells
    frontier_grid = detect_frontiers(grid)
    # Group frontier cells
    _, groups = assign_frontier_groups(frontier_grid)
    # Filter and select top frontier groups
    groups = filter_frontier_groups(groups)
    if not groups:
        # Exploration is complete
        global_goal = None
        return
    else:
        # Find the best frontier to explore
        best_centroid = find_best_frontier_group(groups, current_position)
        if best_centroid is not None:
            # Convert grid indices to world coordinates
            goal_x = best_centroid[1] * resolution + origin_x + resolution / 2.0
            goal_y = best_centroid[0] * resolution + origin_y + resolution / 2.0
            global_goal = (goal_x, goal_y)
        else:
            global_goal = None

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')

        # Initialize TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator()
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Check if the robot is docked
        if self.navigator.getDockedStatus():
            self.get_logger().info("Robot is docked, undocking now.")
            self.navigator.undock()
        else:
            self.get_logger().info("Robot is not docked, proceeding with exploration.")


        # Set up subscriptions
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Variables to store map and odometry data
        self.map_data = None
        self.odom_data = None

        # Flags
        self.map_received = False
        self.odom_received = False
        self.navigation_done = True


        self.timer = self.create_timer(TICK_RATE, self.exploration_loop)

    def exploration_loop(self):
        if not self.map_received or not self.odom_received:
            # Wait until map and odometry data are available
            return

        if self.navigation_done:
            # Get current position in grid coordinates
            current_x = self.x
            current_y = self.y
            current_col = int((current_x - self.origin_x) / self.resolution)
            current_row = int((current_y - self.origin_y) / self.resolution)
            current_position = (current_row, current_col)

            # Run exploration to find the next goal
            occupancy_grid = np.array(self.map_data.data).reshape(self.height, self.width)
            exploration(occupancy_grid, self.resolution, self.origin_x, self.origin_y, current_position)

            if global_goal is None:
                self.get_logger().info("Exploration completed.")
                return
            else:
                goal_x, goal_y = global_goal
                # Create a PoseStamped goal
                self.get_logger().info(f"Navigating to goal at ({goal_x}, {goal_y})")
                goal_pose = self.navigator.getPoseStamped([goal_x, goal_y], TurtleBot4Directions.SOUTH)
                self.navigator.startToPose(goal_pose)
                self.navigation_done = False
        else:
            # Check if navigation is complete
            result = self.navigator.isTaskComplete()
            if result:
                status = self.navigator.getResult()
            if status == 4:  # SUCCEEDED
                self.get_logger().info("Reached the goal successfully.")
                self.navigation_done = True
            else:
                self.get_logger().warn(f"Navigation failed with status {status}.")
                self.navigation_done = True


    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_received = True

    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.odom_received = True


def main(args=None):
    rclpy.init(args=args)
    explorer = AutonomousExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass

    # Shutdown
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
