import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def load_robot_description(robot_description_path, vehicle_params_path):
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()}
    )

    return robot_description.toxml()

def generate_launch_description():
    package_name = "gazebo_ackermann_steering_vehicle"
    package_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(package_path, 'model', 'vehicle.xacro')
    vehicle_params_path = os.path.join(package_path, 'config', 'parameters.yaml')

    robot_urdf = load_robot_description(xacro_file, vehicle_params_path)

    # Launch Arguments
    state_publisher_rename_arg = DeclareLaunchArgument(
        name='description_name',
        default_value='vehicle'
    )

    # Launch Config 
    state_publisher_rename = LaunchConfiguration('description_name')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=state_publisher_rename,
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    return LaunchDescription([
        # Launch Args
        state_publisher_rename_arg,

        # Nodes
        robot_state_publisher_node,
    ])
