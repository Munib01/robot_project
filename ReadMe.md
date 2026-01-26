this is the youtube video i have followed for the designing the robot:
 
https://www.youtube.com/watch?v=wOa1m8hzrgQ


clone the project, then:
- colcon build
- source install/setup.bash
- ros2 launch mobile_robot gazebo_model.launch.py target_x:=20.0 target_y:=0.0

We are facing random craching issues, they are maybe because of VM that I am using.



__________________________________________________________________
Integrate AI:
pip install stable-baseline3 gymnasium shimmy

For reinforcement learning:
run this on a separate terminal:
- ros2 run ros_gz_bridge parameter_bridge /world/maze_world/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean

run this on a separate terminal:
ros2 run mobile_robot train_model

run this on a separate terminal:
- ros2 launch mobile_robot gazebo_model.launch.py
(no target this time)


For run the robot:
run this on a separate terminal:
- ros2 run ros_gz_bridge parameter_bridge /world/maze_world/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean

run this on a separate terminal:
- ros2 run mobile_robot enjoy_robot
this file contains a line "model = PPO.load("maze_bot_model")". this is where we are loading the trained model.
if we are retraining the model and changing the model, we need to change this line as well.

run this on a separate terminal:
- ros2 launch mobile_robot gazebo_model.launch.py target_x:=20.0 target_y:=0.0
