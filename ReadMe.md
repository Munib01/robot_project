this is the youtube video i have followed for the designing the robot:
 
https://www.youtube.com/watch?v=wOa1m8hzrgQ


clone the project, then:
- colcon build
- source install/setup.bash
- ros2 launch mobile_robot gazebo_model.launch.py target_x:=20.0 target_y:=0.0

We are facing random craching issues, they are maybe because of VM that I am using.


things to be added:
- improve the obstacle avoidance
- reinforcement learning
