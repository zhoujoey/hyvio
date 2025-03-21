from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    default_config_path = os.path.join(get_package_share_directory('hyvio'), 'config/euroc_params.yaml')
    if not os.path.exists(default_config_path):
        print(f"Config file {default_config_path} does not exist")
        default_config_path = os.path.join(get_package_share_directory('hyvio'), "config", 'euroc_params.yaml')
    config_path = LaunchConfiguration('config_path', default=default_config_path)
    rviz_config = os.path.join(get_package_share_directory('hyvio'), 'rviz', 'vio_ros2.rviz')
    
    # Create nodes
    vio_node = Node(
        package='hyvio',
        executable='hyvio_node',
        name='vio',
        output='screen',
        parameters=[{
            'config_file': config_path
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='visualization',
        output='log',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        vio_node,
        rviz_node
    ])