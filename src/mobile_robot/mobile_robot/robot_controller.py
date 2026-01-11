import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Create a publisher to the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
       
        # Timer to publish commands every 0.1 seconds (10Hz)
        self.timer = self.create_timer(0.1, self.move_robot)
        self.get_logger().info('Robot Controller Node has started')

    def move_robot(self):
        msg = Twist()
       
        # Set movement values
        msg.linear.x = 0.5   # Forward speed in m/s
        msg.angular.z = -0.2  # Rotation speed in rad/s
       
        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot before shutting down
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()