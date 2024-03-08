import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from time import sleep

class Handler(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node") #Init node
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.th_front = self.th_side = 1.5
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publisher
        self.scan_pub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10) #Laser Scan Subscriber
        self.ok = False
        #self.create_timer(0.1, self.th_callback)
        Thread(target=self.th_callback, daemon=True).start()
        self.rate = self.create_rate(1)

    def th_callback(self):
        while rclpy.ok():
            if self.ok:
                self.cmd_pub.publish(self.msg)

    def rotate(self, clockwise=True):
        factor = -1 if clockwise else 1
        msg = Twist()
        msg.angular.z = 1.0 * factor #rad/s
        self.msg = msg
        self.ok = True
        self.rate.sleep()
        self.ok = False

    def go(self):
        msg = Twist()
        msg.linear.x = 0.31 #m/s
        self.rate.sleep()
        self.cmd_pub.publish(msg)

    def scan_callback(self, msg: LaserScan):
        front = msg.ranges[157]
        right = msg.ranges[0]
        left = msg.ranges[314]
        self.get_logger().debug(f"front: {front}\nright: {right}\nleft: {left}")
        if front > self.th_front and right > self.th_side and left > self.th_side:
            self.go()
        else:
            self.rotate()


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    handller = Handler()
    executor.add_node(handller)

    while rclpy.ok():
        executor.spin()

    handller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()