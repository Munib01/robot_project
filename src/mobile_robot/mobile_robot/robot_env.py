import gymnasium as gym
from gymnasium import spaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.srv import ControlWorld # For Reset
import numpy as np
import math

class RobotEnv(gym.Env, Node):
    def __init__(self):
        gym.Env.__init__(self)
        Node.__init__(self, 'robot_env_node')

        # Actions: [Linear X speed, Angular Z speed]
        self.action_space = spaces.Box(low=np.array([0.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        # Obs: 10 Lidar points + dist_to_goal + angle_to_goal
        self.observation_space = spaces.Box(low=0.0, high=12.0, shape=(12,), dtype=np.float32)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_client = self.create_client(ControlWorld, '/world/maze_world/control')
        
        # Subscriptions to update state
        self.create_subscription(LaserScan, '/scan', self._lidar_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.scan_data = np.ones(10) * 10.0
        self.pose = [0.0, 0.0, 0.0]
        self.target = [20.0, 0.0]

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 1. Reset Gazebo
        req = ControlWorld.Request()
        req.world_control.reset.all = True
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset service...')
        self.reset_client.call_async(req)
        
        # 2. Wait for physics to settle
        import time
        time.sleep(0.5)
        return self._get_obs(), {}

    def step(self, action):
        # 1. Apply Action from RL Agent
        msg = Twist()
        msg.linear.x = float(action[0] * 0.5)
        msg.angular.z = float(action[1] * 1.0)
        self.publisher_.publish(msg)

        # 2. Spin ROS to get new data
        rclpy.spin_once(self, timeout_sec=0.1)

        # 3. Get results
        obs = self._get_obs()
        reward, done = self._calculate_reward()
        return obs, reward, done, False, {}

    def _get_obs(self):
        dist = math.sqrt((self.target[0]-self.pose[0])**2 + (self.target[1]-self.pose[1])**2)
        angle = math.atan2(self.target[1]-self.pose[1], self.target[0]-self.pose[0]) - self.pose[2]
        angle = math.atan2(math.sin(angle), math.cos(angle))
        return np.append(self.scan_data, [dist, angle]).astype(np.float32)

    def _calculate_reward(self):
        if min(self.scan_data) < 0.6: return -100.0, True # Crash
        dist = math.sqrt((self.target[0]-self.pose[0])**2 + (self.target[1]-self.pose[1])**2)
        if dist < 0.5: return 500.0, True # Goal
        return -0.1, False # Living penalty

    # (Lidar and Odom callbacks stay the same as your current controller)
    def _lidar_cb(self, msg):
        if msg.ranges:
            # Downsample 720 rays to 10 rays for the observation space
            indices = np.linspace(0, len(msg.ranges) - 1, 10, dtype=int)
            self.scan_data = np.array([
                msg.ranges[i] if (not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0) 
                else 12.0 for i in indices
            ])

    def _odom_cb(self, msg):
        # Position
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        
        # Yaw (Theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.pose[2] = math.atan2(siny_cosp, cosy_cosp)