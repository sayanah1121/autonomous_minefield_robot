import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # Publishers & Subscribers
        self.path_pub = self.create_publisher(Path, 'safe_path', 10)
        self.create_subscription(OccupancyGrid, 'minefield_map', self.map_callback, 10)
        
        self.map_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.get_logger().info("A* Planner Node Started")

    def map_callback(self, msg):
        """
        Triggered whenever the map updates. 
        In a real scenario, you might trigger this on a separate service call 
        or when a goal is clicked in Rviz.
        """
        self.map_data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Define Start (Robot Current) and Goal (Target)
        # For demo purposes, we hardcode these. 
        # Ideally, start = current odometry, goal = user input (e.g. /goal_pose)
        start_world = (0.0, 0.0) 
        goal_world = (2.0, 2.0)  # 2 meters away
        
        # Convert World Coords (meters) to Grid Coords (indices)
        start_grid = self.world_to_grid(start_world)
        goal_grid = self.world_to_grid(goal_world)

        if self.is_valid(start_grid) and self.is_valid(goal_grid):
            path_grid = self.run_astar(start_grid, goal_grid)
            if path_grid:
                self.publish_path(path_grid)
        else:
            self.get_logger().warn("Start or Goal is out of map bounds or inside an obstacle.")

    def run_astar(self, start, goal):
        """
        The Core A* Algorithm
        start, goal: tuples (x, y) in grid coordinates
        """
        # Priority Queue: Stores (f_score, (x, y))
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        came_from = {} # To reconstruct the path
        g_score = {start: 0} # Cost from start to current
        
        # f_score = g_score + heuristic
        f_score = {start: self.heuristic(start, goal)}
        
        while open_list:
            # Pop node with lowest f_score
            _, current = heapq.heappop(open_list)
            
            # Reached Goal?
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            # Check Neighbors (Up, Down, Left, Right)
            neighbors = [
                (current[0] + 1, current[1]),
                (current[0] - 1, current[1]),
                (current[0], current[1] + 1),
                (current[0], current[1] - 1)
            ]
            
            for neighbor in neighbors:
                if not self.is_valid(neighbor):
                    continue
                
                # Assume cost to move to neighbor is 1
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f = tentative_g_score + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_list, (f, neighbor))
                    
        self.get_logger().info("No path found!")
        return None

    def heuristic(self, a, b):
        # Manhattan distance for grid
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse() # Start to Goal
        return path

    def is_valid(self, node):
        x, y = node
        
        # 1. Check Map Bounds
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        
        # 2. Check Obstacles (Mines)
        # Map data is 1D array. Index = y * width + x
        index = y * self.width + x
        probability = self.map_data[index]
        
        # -1 is Unknown, 0 is Free, 100 is Occupied
        # We treat anything > 50 as an obstacle
        if probability > 50:
            return False
            
        return True

    def world_to_grid(self, world_coords):
        wx, wy = world_coords
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return (gx, gy)

    def grid_to_world(self, grid_coords):
        gx, gy = grid_coords
        wx = (gx * self.resolution) + self.origin_x
        wy = (gy * self.resolution) + self.origin_y
        return (wx, wy)

    def publish_path(self, grid_path):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for grid_point in grid_path:
            pose = PoseStamped()
            pose.header = msg.header
            wx, wy = self.grid_to_world(grid_point)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0 # No rotation needed for path points
            msg.poses.append(pose)
            
        self.path_pub.publish(msg)
        self.get_logger().info(f"Published path with {len(grid_path)} steps.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
