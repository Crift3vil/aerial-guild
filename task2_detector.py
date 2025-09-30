#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
import math
import time

class Detector(Node):
    def __init__(self):
        super().__init__('tdetector')

        self.sub_reset = self.create_subscription(String, 'reset_collided', self.reset_collided, 10)
        self.sub_player = self.create_subscription(Pose, 'turtle1/pose', self.update_player, 10)
        self.sub_prey = self.create_subscription(Pose, 'prey/pose', self.update_prey, 10)
        self.pub = self.create_publisher(String, 'turtle_caught', 10)
        self.collided = False
        self.player_pose = None
        self.prey_pose = None

    def update_player(self, msg):
        self.player_pose = msg

    def update_prey(self, msg):
        self.prey_pose = msg

    def check_collision(self):
        if self.player_pose and self.prey_pose and not self.collided:
            dist = math.dist((self.player_pose.x, self.player_pose.y),
                             (self.prey_pose.x, self.prey_pose.y))
            if dist < 0.5:
                self.pub.publish(String(data='turtle_caught'))
                self.get_logger().info("Caught prey!")
                self.collided = True
    def reset_collided(self, msg):
        self.collided = False
        self.get_logger().info("reset received")

        
                

def main(args = None):
    rclpy.init(args=args)
    node = Detector()
    timer = node.create_timer(0.5, node.check_collision)  
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
