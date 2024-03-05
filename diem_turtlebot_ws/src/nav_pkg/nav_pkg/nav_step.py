import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class Handler(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node") #Init node
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.rate_stop = self.create_rate(1/4)
        self.th_front = self.th_side = 1.5
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publisher
        self.ok = False

    def move_callback(self):
        while rclpy.ok():
            if self.ok:
                self.cmd_pub.publish(self.msg)

    def rotate(self, clockwise=True):
        rate = self.create_rate(1)
        factor = -1 if clockwise else 1
        msg = Twist()
        msg.angular.z = 1.57 * factor #rad/s
        self.msg = msg
        self.ok = True
        rate.sleep()
        self.ok = False

    def go(self):
        rate = self.create_rate(1/4)
        msg = Twist()
        msg.linear.x = 0.25 #m/s
        self.msg = msg
        self.ok = True
        rate.sleep()
        self.ok = False
    
    def loop(self):
        self.get_logger().info("Node starting...")
        #self.rate_stop.sleep()
        self.get_logger().info("Moving forward...")
        self.go()
        #self.rate_stop.sleep()
        self.get_logger().info("Rotating...")
        self.rotate()
        #self.rate_stop.sleep()
        self.get_logger().info("Moving forward...")
        self.go()
        self.get_logger().info("End!")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    handller = Handler()
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    handller.set_parameters([param])
    executor.add_node(handller)
    executor.create_task(handller.loop)
    executor.create_task(handller.move_callback)
    executor.spin()

    handller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()