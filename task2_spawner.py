#!/usr/bin/env python3 
import time
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn,Kill
import random
from std_msgs.msg import String
class TurtleManager(Node):
    def __init__(self):
        super().__init__("tspawner")
        self.kill_count =0
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, "kill")
        self.sub = self.create_subscription(String, 'turtle_caught', self.caught, 10)
        self.pub_reset = self.create_publisher(String, 'reset_collided', 10)
        

        self.spawn_new_turtle()


    def spawn_new_turtle(self):
        req = Spawn.Request()
        req.x = random.uniform(1.0,10.0)
        req.y = random.uniform(1.0,10.0)
        req.theta = 0.0
        req.name = 'prey'
        self.spawn_client.call_async(req)

    def caught(self, msg):
        req2 = Kill.Request()
        req2.name = "prey" 
        self.kill_client.call_async(req2)
        reset_msg = String()
        reset_msg.data = "reset_collided"
        self.pub_reset.publish(reset_msg)
        self.kill_count+=1
        time.sleep(0.5)
        self.spawn_new_turtle()
        self.get_logger().info(str(self.kill_count))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
