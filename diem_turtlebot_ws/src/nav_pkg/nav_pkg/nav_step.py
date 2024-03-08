import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.clock import Duration

class Handler(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node") #Init node
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publisher
        self.ok = False

    def move_callback(self):
        while rclpy.ok():
            if self.ok:
                self.cmd_pub.publish(self.msg)

    def rotate(self, clockwise=True):
        factor = -1 if clockwise else 1
        msg = Twist()
        msg.angular.z = 1.57 * factor #rad/s
        self.msg = msg
        self.ok = True
        self.sleep(1)
        self.ok = False

    def go(self):
        msg = Twist()
        msg.linear.x = 0.25 #m/s
        self.msg = msg
        self.ok = True
        self.sleep(4)
        self.ok = False

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0 #m/s
        self.msg = msg
        self.ok = True
        self.sleep(1)
        self.ok = False
    
    def sleep(self, time_seconds):
        self.get_clock().sleep_for(Duration(seconds=time_seconds)) #sleep for <time_seconds> seconds
    
    def loop(self):
        self.get_logger().info("Node starting...")
        self.sleep(1)
        self.get_logger().info("Moving forward...")
        self.go() #Moving forward for 4 seconds
        self.get_logger().info("Rotating...")
        self.rotate() #Rotating for 1 seconds
        self.get_logger().info("Moving forward...")
        self.go() #Move forward for 4 seconds
        self.get_logger().info("Stopping...")
        self.stop()
        self.get_logger().info("End!")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    handller = Handler()
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    handller.set_parameters([param])
    executor.add_node(handller)
    executor.create_task(handller.move_callback) #Run a thread with callable object given as input
    executor.create_task(handller.loop)
    try:
        executor.spin() #Running loop - bocking call
    except KeyboardInterrupt:
        pass

    handller.destroy_node()

if __name__ == "__main__":
    main()