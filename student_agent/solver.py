"""
Write your own solver in the scan_callback function
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

TOP_SPEED = 10
ACCELARATION = 6
TURN_SPEED = 7
SENSOR_RANGE = 7

class StudentSolver(Node):
    def __init__(self):
        super().__init__('student_solver')
        self.state = "moving"
        
        # subscriber to read sensor values (L,F,R)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )
        
        # publisher to send movement commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )
        
        self.get_logger().info("Student Solver Node initialized successfully.")
        self.get_logger().info(f"Stats -> Speed: {TOP_SPEED}, Accel: {ACCELARATION}, Turn: {TURN_SPEED}, Range: {SENSOR_RANGE}")

    def scan_callback(self, msg):

        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]
    
        cmd = Twist()
    
        if self.state == "moving":
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
    
            if d_front < 0.65:
                self.state = "turning"
            if d_left < 0.3:
                cmd.angular.z = -0.5
            if d_right < 0.3:
                cmd.angular.z = 0.5
    
        elif self.state == "turning":
            cmd.linear.x = 0.0
            cmd.angular.z = -1.5
    
            if d_front > 0.8:
                self.state = "moving"
    
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StudentSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

