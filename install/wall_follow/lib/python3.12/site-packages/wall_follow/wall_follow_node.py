
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from rclpy.qos import qos_profile_sensor_data

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow')
        self.get_logger().info('WallFollowNode init')
        self.pub = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, '/diff_drive/scan', self.scan_cb, qos_profile_sensor_data)

        # thresholds & speeds
        self.front_th      = 5.5   # m
        self.left_near     = 7.0  # m
        self.left_far      = 8.5  # m
        self.forward_speed = 0.3   # m/s
        self.turn_speed    = 0.7   # rad/s

        self.wall_found = False

    def scan_cb(self, msg):
        # extract front & left readings
        if len(msg.ranges)==1 or math.isnan(msg.angle_increment):
            front = left = msg.ranges[0]
        else:
            n = len(msg.ranges)
            fi = int((0.0       - msg.angle_min)   / msg.angle_increment)
            li = int((math.pi/2 - msg.angle_min)   / msg.angle_increment)
            fi = max(0, min(fi, n-1))
            li = max(0, min(li, n-1))
            front = msg.ranges[fi]
            left  = msg.ranges[li]

        cmd = Twist()

        # Forward until wall spotted
        if not self.wall_found:
            cmd.linear.x = self.forward_speed
            if left < self.left_far:
                self.wall_found = True
                self.get_logger().info('Left wall detected, entering FSM')
            self.pub.publish(cmd)
            return

        # FSM transitions
        if front < self.front_th or left <= self.left_near:
            # Rotate Right
            cmd.angular.z = -self.turn_speed

        elif front >= self.front_th and left >= self.left_far:
            # Rotate Left
            cmd.angular.z = +self.turn_speed

        else:
            # Otherwise Forward
            cmd.linear.x = self.forward_speed

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
