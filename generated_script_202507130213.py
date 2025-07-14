import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscribers for odometry and lidar
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Timer to publish at 10Hz
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # State variables
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laser_ranges = []

        # Zigzag parameters
        self.zig_right = True
        self.zig_timer = 0.0
        self.zig_duration = 5.0  # seconds per zig or zag
        self.cycle_count = 0
        self.max_cycles = 10

        # Return-to-start flag
        self.returning = False

    def odom_callback(self, msg: Odometry):
        # Update current pose
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        # Extract yaw from quaternion
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 * (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        # Record start position once
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

    def scan_callback(self, msg: LaserScan):
        self.laser_ranges = msg.ranges

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def timer_callback(self):
        twist = Twist()

        # Obstacle detection: look ahead at +-10 degrees
        obstacle_ahead = False
        if self.laser_ranges:
            mid = len(self.laser_ranges) // 2
            window = self.laser_ranges[mid-10:mid+10]
            if any(r < 0.5 for r in window if r > 0.0):
                obstacle_ahead = True

        if not self.returning:
            # Zigzag loop
            if obstacle_ahead:
                # simple avoidance: rotate in place
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            else:
                # increment zig timer
                self.zig_timer += self.timer_period
                if self.zig_timer >= self.zig_duration:
                    self.zig_timer = 0.0
                    self.zig_right = not self.zig_right
                    self.cycle_count += 1
                    if self.cycle_count >= self.max_cycles:
                        self.returning = True
                # set zigzag motion
                twist.linear.x = 0.3
                twist.angular.z = 0.2 if self.zig_right else -0.2
        else:
            # Return to start
            dx = self.start_x - self.current_x
            dy = self.start_y - self.current_y
            dist = math.hypot(dx, dy)
            if dist < 0.1:
                # arrived at start: stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # point toward start
                target_yaw = math.atan2(dy, dx)
                yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
                if abs(yaw_error) > 0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5 * math.copysign(1.0, yaw_error)
                else:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0

        # Publish command
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()