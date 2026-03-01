import heapq
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from tf2_ros import Buffer, TransformListener

GridXY = Tuple[int, int]

class PlannerNode(Node):
    def __init__(self):
        super().__init__("Simple_planner")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg: Optional[OccupancyGrid] = None
        self.width: int = 0
        self.height: int = 0
        self.resolution: float = 0.0
        self.origin_x: float = 0.0
        self.origin_y: float = 0.0
        self.grid: Optional[List[int]] #the flat list containing occupancy probabilities

        map_qos = QoSProfile(
            depth = 1, 
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        # Subscribe to message type "map" under topic OccupancyGrid under the alias map_sub. 
        # Callback function is on_map function, meaning on_map gets executed every time a message comes in
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos) 

        self.goal_sub_1 = self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.goal_sub_2 = self.create_subscription(PoseStamped, "/move_base_simple/goal", self.on_goal, 10)

        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("occupied_threshold", "50")
        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("allow_diagonal", True)

        self.get_logger().info("Simple planner ready. Waiting for /map and a goal")
        
        # CALLBACKS

    def on_map(self: Node, msg: OccupancyGrid):
        self.map_msg = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution= msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.grid = list(msg.data)
        
        self.get_logger().info(
            f"Got map: {self.width}x{self.height}, res={self.resolution:.3f}, "
            f"origin=({self.origin_x:.2f},{self.origin_y:.2f})"
        )

    def on_goal(self: Node, goal_msg: PoseStamped):
        if self.map_msg is None or self.grid is None:
            self.get_logger().warn("No map yet, can't plan.")
            return

        global_frame = self.get_parameter("global_frame").value
        base_frame = self.get_parameter("base_frame").value

        # 1) Get robot start pose in map frame at planning time (using TF lookup)
        start_world = self.lookup_robot_xy(global_frame, base_frame)
        if start_world is None:
            self.get_logger().warn("TF start pose is not available (map -> base). Can't plan!")

        start_x, start_y = start_world # start pose in world coordinates
        
        # 2) Read goal pose (assuming it is in map frame)
        
        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y

        # 3) convert world to grid

        start = self.world_to_grid(start_x, start_y)
        goal = self.world_to_grid(goal_x, goal_y)

        if start is None or goal is None:
            self.get_logger().warn("Start or goal outside map bounds")
            return
        
        if not self.is_free(*start) or not self.is_free(*goal):
            self.get_logger().warn("Start or goal is in occupied or unknown cell.")
            return
        
        self.get_logger().info(
            f"Planning: start(grid)={start} goal(grid)={goal} "
            f"start(world)=({start_x:.2f},{start_y:.2f}) goal(world)=({goal_x:.2f},{goal_y:.2f})"
        )

        # 4) Run A*
        path_cells = self.astar(start, goal)
        if path_cells is None:
            self.get_logger().warn("No path found.")
            return
        
        # 5) Publish nav_msgs/Path for RViz
        path_msg = self.make_path_msg(path_cells, frame_id = global_frame)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_msg.poses)} poses on /planned_path")

    # TF helper functions

    def lookup_robot_xy(self: Node, global_frame: str, base_frame: str) -> Optional[Tuple[float]]:
        try:
            # query at planning time
            t  = self.tf_buffer.lookup_transform(global_frame, base_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {e}")
            return None
        
    # Map/Grid helper functions

    def world_to_grid(self: Node, wx: float, wy: float) -> Optional[GridXY]:
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return (gx, gy)
    
    def grid_to_world(self: Node, gx: int, gy: int) -> Tuple[float, float]:
        wx = gx * self.resolution + self.origin_x + 0.5 * self.resolution
        wy = gy * self.resolution + self.origin_y + 0.5 * self.resolution
        return (wx, wy)
    
    def idx(self: Node, x: int, y: int) -> int: #will use this to flatten the grid into an occupancy probability row vector i think
        return y * self.width + x 
    
    def is_free(self: Node, x: int, y: int) -> bool:
        val = self.grid[self.idx(x, y)]
        occ_thresh = self.get_parameter("occupied_threshold").get_parameter_value().integer_value
        unknown_is_obs = (
            self.get_parameter("treat unknown as obstacle").get_parameter_value().bool_value
        )

        if val < 0:
            return not unknown_is_obs
        return val < occ_thresh
    
    # A* implementation

    def astar(self: Node, start: GridXY, goal: GridXY) -> Optional[list[GridXY]]: #optional means that this function outputs list only if a list is returned 
        allow_diag = self.get_parameter("allow_diagonal").get_parameter_value().bool_value
        neighbours = self.get_neighbours8 if allow_diag else self.get_neighbours4

        open_heap: List[Tuple[float, GridXY]] = []
        heapq.heappush(open_heap, (0.0, start))

        came_from: dict[GridXY, GridXY] = {}
        g_score: dict[GridXY, float] = {start: 0.0}

        def h(a: GridXY, b: GridXY) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])
        
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current is closed:
                continue
            closed.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current) #this will contain the path and will be returned IF a path is found. o.w. none is outputted
            
            for nb, step_cost in neighbours(current):
                if nb in closed:
                    continue
                if not self.is_free(nb[0], nb[1]):
                    continue

                tentative_g = g_score[current] + step_cost
            
                if tentative_g < g_score.get(nb, float("inf")):
                    came_from[nb] = current
                    g_score[nb] = tentative_g
                    f = tentative_g + h(nb, goal)
                    heapq.heappush(open_heap, (f, nb))

        return None
    
    def get_neighbours4(self: Node, node: GridXY) -> list[tuple[GridXY, float]]:
        x, y = node
        out = []
        for dx, dy in [(1,0), (-1, 0), (0,1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny <self.height:
                out.append(((nx, ny), 1.0))
        return out
    
    def get_neighbours8(self: Node, node: GridXY) -> list[tuple[GridXY, float]]:
        x, y = node
        out = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny <self.height:
                        cost = math.sqrt(2) if (dx != 0 and dy != 0) else 1.0
                        out.append(((nx, ny), cost))
                return out
            
    def reconstruct_path(self: Node, came_from: Dict[GridXY, GridXY], current: GridXY) -> List[GridXY]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    # Publish helpers

    def make_path_msg(self: Node, cells: List[GridXY], frame_id: str) -> Path:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for (gx, gy) in cells:
            wx, wy = self.grid_to_world(gx, gy)
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = path.header.stamp
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        return path
        
def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
