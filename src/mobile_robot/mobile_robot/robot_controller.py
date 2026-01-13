import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # 1. Publishers & Subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Declare ROS 2 parameters (Target Coordinates)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.goal_tolerance = 0.5

        # Get the values
        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value
                
        # 3. State Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.min_obstacle_distance = 10.0
        self.state = "SEEK_GOAL"
        self.avoidance_start_time = 0.0
        
        # 4. Control Timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Controller Started. Goal: ({self.target_x}, {self.target_y})')

    def odom_callback(self, msg):
        """Extracts position and orientation (Yaw) from Odometry."""
        # Position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Orientation (Quaternion to Euler Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        """Standard LiDAR safety check."""
        if msg.ranges:
            front_sector = msg.ranges[300:420]
            clean_ranges = [r if not math.isinf(r) else 10.0 for r in front_sector]
            self.min_obstacle_distance = min(clean_ranges)
            
            # self.min_obstacle_distance = min(msg.ranges)
            self.get_logger().info(f'min_obstacle_distance: ({self.min_obstacle_distance})')

    def control_loop(self):
        msg = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]

        # A. Calculate Euclidean distance
        dist_x = self.target_x - self.current_x
        dist_y = self.target_y - self.current_y
        distance_to_goal = math.sqrt(dist_x**2 + dist_y**2)
        # self.get_logger().info(f'current: ({self.current_x}, {self.current_y})')
        # self.get_logger().info(f'target: ({self.target_x}, {self.target_y})')

        # B. Calculate Angle Error
        angle_to_target = math.atan2(dist_y, dist_x)
        angle_error = angle_to_target - self.current_theta
        
        # Normalize angle_error to [-pi, pi] to prevent infinite spinning
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('SUCCESS: Goal Reached!')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return  # Exit the loop entirely so it stops moving

        # C. State Machine logic
        if self.state == "SEEK_GOAL":
            if self.min_obstacle_distance < 1.2: # Trigger distance
                self.state = "AVOID_ROTATE"
            else:
                if abs(angle_error) > 0.2:
                    msg.angular.z = 0.6 if angle_error > 0 else -0.6
                else:
                    msg.linear.x = 0.5
        
        elif self.state == "AVOID_ROTATE":
            self.get_logger().warn('ROTATING TO CLEAR...')
            # Rotate until front is very clear
            if self.min_obstacle_distance < 2.5:
                msg.angular.z = 0.7 
            else:
                self.state = "AVOID_DRIVE"
                self.avoidance_start_time = now
                
        elif self.state == "AVOID_DRIVE":
            self.get_logger().info('DRIVING PAST OBSTACLE...')
            # Drive for 5 seconds (as you set) to fully clear the wall
            if (now - self.avoidance_start_time) < 5.0:
                msg.linear.x = 0.4
                msg.angular.z = 0.0
            else:
                self.state = "SEEK_GOAL"

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send zero velocity on shutdown
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
