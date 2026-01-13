import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():
	# this name must match robot name in xacro file
	robotXacroName='differential_drive_robot'


	# Declare the arguments
	target_x_arg = DeclareLaunchArgument('target_x', default_value='5.0')
	target_y_arg = DeclareLaunchArgument('target_y', default_value='5.0')

	namePackage = 'mobile_robot'
	
	#path to you maze file
	world_file_name = 'maze_1.sdf'
	world_path = os.path.join(get_package_share_directory(namePackage), 'worlds', world_file_name)

	modelFileRelativePath = 'model/robot.xacro'

	pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

	robotDescription = xacro.process_file(pathModelFile).toxml()

	gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))

	gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args':[f'-r -v4 {world_path}'], 'on_exit_shutdown': 'true'}.items())

	# Gazebo node
	spawnModelNodeGazebo = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=[
			'-name', robotXacroName,
			'-topic', 'robot_description'
		],
		output='screen',
	)

	# Robot State Publisher Node
	nodeRobotStatePublisher = Node (
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description':robotDescription,
		'use_sim_time':True}]
	)

	# Add the node with parameters
	nodeRobotController = Node(
		package='mobile_robot',
		executable='robot_controller',
		output='screen',
		parameters=[{
			'use_sim_time': True,
			'target_x': LaunchConfiguration('target_x'),
			'target_y': LaunchConfiguration('target_y'),
		}]
	)

	# this is to control robot from ROS2
	bridge_params = os.path.join(
	get_package_share_directory(namePackage),
	'parameters',
	'bridge_parameters.yaml'
	)

	start_gazebo_ros_bridge_cmd = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		arguments=[
		'--ros-args',
		'-p',
		f'config_file:={bridge_params}',
	],
	output='screen',
	)

	launchDescriptionObject = LaunchDescription()

	launchDescriptionObject.add_action(gazeboLaunch)

	launchDescriptionObject.add_action(spawnModelNodeGazebo)
	launchDescriptionObject.add_action(nodeRobotStatePublisher)
	launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
	launchDescriptionObject.add_action(target_x_arg)
	launchDescriptionObject.add_action(target_y_arg)
	launchDescriptionObject.add_action(nodeRobotController)

	return launchDescriptionObject
