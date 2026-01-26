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
        
        # 2. Parameters
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.goal_tolerance = 0.5

        # 3. State Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Three-view LiDAR variables
        self.view_left = 10.0
        self.view_front = 10.0
        self.view_right = 10.0
        self.min_obstacle_distance = 10.0
        
        self.state = "SEEK_GOAL"
        self.avoidance_start_time = 0.0
        self.turn_direction = 1.0 # 1.0 for Left, -1.0 for Right
        
        # 4. Control Timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Controller Started. Goal: ({self.target_x}, {self.target_y})')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        if msg.ranges:
            # Clean helper: handles 'inf' and ensures we ignore self-reflections < 0.8m
            def get_min_range(data):
                filtered = [r for r in data if r > 0.8 and not math.isinf(r)]
                return min(filtered) if filtered else 10.0

            # Split 720 rays into three 120-degree sectors
            # Indices: Right (0-239), Front (240-479), Left (480-719)
            # self.view_right = get_min_range(msg.ranges[60:180])
            # self.view_front = get_min_range(msg.ranges[300:420])
            # self.view_left = get_min_range(msg.ranges[540:660])


            self.view_right = get_min_range(msg.ranges[60:180])
            self.view_front = get_min_range(msg.ranges[300:420])
            self.view_left = get_min_range(msg.ranges[540:660])
            self.view_all = get_min_range(msg.ranges[0:719])
            
            self.min_obstacle_distance = self.view_front
            # self.get_logger().info(f'Distances: ({round(self.view_front,2)}, {round(self.view_right, 2)}, {round(self.view_left,2)})')


    def control_loop(self):

        msg = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]

        # A. Navigation Math
        dist_x = self.target_x - self.current_x
        dist_y = self.target_y - self.current_y
        distance_to_goal = math.sqrt(dist_x**2 + dist_y**2)
        self.get_logger().info(f'Distances: ({round(distance_to_goal,2)}')


        angle_to_target = math.atan2(dist_y, dist_x)
        angle_error = angle_to_target - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # B. Priority 1: Goal Check
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('GOAL REACHED!')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return

        # C. Priority 2: State Machine Navigation
        if self.state == "SEEK_GOAL":
            self.get_logger().info('SEEK_GOAL!')

            if self.view_front < 1.2:
                # Decide turn direction once based on side with more space
                self.turn_direction = 0.1 if self.view_left > self.view_right else -0.1
                self.state = "AVOID_ROTATE"
            else:
                # Smooth navigation
                if abs(angle_error) > 0.3:
                    msg.angular.z = 0.6 if angle_error > 0 else -0.6
                    msg.linear.x = 0.1 # Slow creep while turning
                else:
                    msg.linear.x = 0.5

        elif self.state == "AVOID_ROTATE":
            self.get_logger().info('AVOID_ROTATE!')

            # Rotate until the front is completely clear
            if self.view_front < 1.4:
                msg.angular.z = self.turn_direction
                msg.linear.x = 0.0
            else:
                self.avoidance_start_time = now
                self.state = "AVOID_DRIVE"

        elif self.state == "AVOID_DRIVE":
            self.get_logger().info('AVOID_DRIVE!')
            # Commit to driving straight to clear the corner
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
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()