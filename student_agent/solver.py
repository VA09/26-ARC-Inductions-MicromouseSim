"""
Write your own solver in the scan_callback function
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

TOP_SPEED = 10
ACCELARATION = 6
TURN_SPEED = 5
SENSOR_RANGE = 9

class StudentSolver(Node):
    def __init__(self):
        super().__init__('student_solver')
        self.state = "moving"
        self.direction = None
        self.turn_count = 0
        
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

        self.side = "left"
        self.prev_error = 0.0
        self.stuck_count = 0
        self.open_count = 0
        self.last_scan = None
        self.turn_count = 0
        self.turn_direction = 0

    def scan_callback(self, msg):

        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]

        TARGET = 0.5
        F_STOP = 0.65
        F_SLOW = 1.3
        SIDE_OPEN = 0.8
        MAX_LIN = 0.5
        MAX_ANG = 1.5 
        KP,KD = 3.0,1.0
    
        cmd = Twist()

        def clip(v,lo,hi):
            return max(lo,min(v,hi))
        if self.side == "right":
            s = 1.0
            d_side = d_right
        else:
            s=-1.0
            d_side = d_left
        if self.last_scan is None:
            self.last_scan = (d_left, d_front, d_right)
            return

        drift = (abs(d_left-self.last_scan[0]) + abs(d_front-self.last_scan[1]) + abs(d_right-self.last_scan[2]))
        self.last_scan = (d_left,d_front,d_right)
        if self.turn_count > 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5
            self.turn_count -= 1
        if drift<0.02:
            self.stuck_count = self.stuck_count +1 
        if self.stuck_count>15:
            cmd.linear.x = -0.15*MAX_LIN
            cmd.angular.z = -s*MAX_ANG*0.85
            if self.stuck_count>65:
                self.stuck_count = 0
        if d_side > SIDE_OPEN:
            cmd.linear.x = 0.35 * MAX_LIN
            cmd.angular.z = -s * MAX_ANG * 0.5

        if d_front>F_STOP:
            cmd.linear.x = 0.5 * MAX_LIN

        elif d_front < F_STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = s * MAX_ANG

        else:
            error = s*(TARGET-d_side)
            deriv = error - self.prev_error
            self.prev_error = error
            cmd.angular.z = clip(KP * error + KD * deriv, -MAX_ANG, MAX_ANG)
            speed_scale = clip(d_front / F_SLOW, 0.25, 1.0)
            speed_scale *= (1.0 - 0.4 * abs(cmd.angular.z) / MAX_ANG)
            cmd.linear.x = MAX_LIN * speed_scale
        if self.open_count > 15:
            cmd.linear.x *=0.25
        cmd.linear.x  = clip(cmd.linear.x, -0.2 * MAX_LIN, MAX_LIN)
        cmd.angular.z = clip(cmd.angular.z, -MAX_ANG, MAX_ANG)
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

