import rclpy
import heapq 
import math
from typing import Dict, List, optional, Tuple

import rclpy
from geometry_msgs import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from tf2_ros import Buffer, TransformListener

GridXY = Tuple[int, int]

class PlannerNode(Node): #class of type Node, it is a ROS2 inbuilt class
    def __init__(self):
        super().__init__("simple_planner") #super() calls the init (constructor) of the parent class, meaning ROS's 
                                           #inbuilt "node" class in this case
        self.get_logger().info("simple planner node started") #simple printer
        self.timer = self.create_timer(2.0, self.on_timer) #create_timer will call the given function every given time
                                                           #in this case, on_timer will be called every 2 seconds

    def on_timer(self): #this function checks if map is present (every 2 seconds due to the self.timer above)
        topic_names = [name for (name, _types) in self.get_topic_names_and_types()]
        has_map = "/map" in topic_names
        self.get_logger().info(f"Alive. /map present? {has_map}")

def main():
    rclpy.init() #initializes ROS2 python client library
    node = PlannerNode() #constructs a PlannerNode
    try:
        rclpy.spin(node) #keep checking for callbacks and handle them unless keyboard interrupt happens
                         #checking for a callback is essentially waiting for a trigger that will automatically execute an event
                         #in our case, we told ROS every 2 seconds, CALL self.on_timer
                         #spin just makes it so that ROS keeps listening for that callback and keeps handling it
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
