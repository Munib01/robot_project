import rclpy
from rclpy.node import Node
from stable_baselines3 import PPO
from mobile_robot.robot_env import RobotEnv # Reuse your observation logic
import time

def main(args=None):
    rclpy.init(args=args)
    
    # 1. Create the environment (the "body")
    # Note: We use the same RobotEnv so the sensors are processed exactly the same way
    env = RobotEnv()

    # 2. Load the trained "brain"
    # Make sure the filename matches what you saved at the end of training
    model = PPO.load("maze_bot_model")
    print("Model loaded! The robot is now using its artificial intelligence.")

    obs, _ = env.reset()
    
    try:
        while rclpy.ok():
            # 3. Ask the model for the best action based on current sensors
            # deterministic=True makes the robot follow its best path without 'exploring'
            action, _states = model.predict(obs, deterministic=True)

            # 4. Apply the action to the environment
            obs, reward, done, truncated, info = env.step(action)

            if done or truncated:
                print("Goal reached or collision detected. Resetting...")
                obs, _ = env.reset()
                
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()