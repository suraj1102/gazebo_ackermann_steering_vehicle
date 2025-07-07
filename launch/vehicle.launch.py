import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)

from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()}
    )

    return robot_description.toxml()


def start_vehicle_control():
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)


def generate_launch_description():
    # Launch arguments
    world_arg = DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world'),
            description='Gazebo world file'
        )
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.3')
    roll_arg = DeclareLaunchArgument('R', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('P', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0')

    # Launch config
    world_file = LaunchConfiguration('world')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('R'), LaunchConfiguration('P'), LaunchConfiguration('Y')

    # Paths
    package_name = "gazebo_ackermann_steering_vehicle"
    package_path = get_package_share_directory(package_name)
    robot_description_path = os.path.join(package_path, 'model', 'vehicle.xacro')
    vehicle_params_path = os.path.join(package_path, 'config', 'parameters.yaml')

    # Load URDF
    robot_description = load_robot_description(robot_description_path, vehicle_params_path)
    robot_name = "ackermann_steering_vehicle"

    # Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', world_file,
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen'
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    # Controllers
    joint_state, forward_velocity, forward_position = start_vehicle_control()

    # Vehicle controller node
    vehicle_controller_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='vehicle_controller',
        parameters=[vehicle_params_path],
        output='screen'
    )

    return LaunchDescription([
        # Launch order: spawn → joint_state → velocity → position
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[joint_state])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state, on_exit=[forward_velocity, forward_position])),

        # Arguments
        world_arg, x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,

        # Gazebo
        start_gazebo_cmd,

        # Nodes
        spawn_entity,
        robot_state_publisher_node,
        vehicle_controller_node
    ])
