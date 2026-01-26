import rclpy
from stable_baselines3 import PPO
from mobile_robot.robot_env import RobotEnv

def main():
    rclpy.init()
    env = RobotEnv()
    # Training the "Brain"
    model = PPO.load("maze_bot_model", env=env,)
    model.learn(total_timesteps=50000)
    model.save("maze_bot_model_v2")
    rclpy.shutdown()