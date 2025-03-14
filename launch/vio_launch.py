from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    fixed_frame_id_arg = DeclareLaunchArgument(
        'fixed_frame_id',
        default_value='world'
    )

    # Get package share directory
    pkg_dir = get_package_share_directory('larvio')
    
    # Create nodes
    vio_node = Node(
        package='larvio',
        executable='larvio_node',
        name='vio',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_dir, 'config/euroc.yaml'),
            'fixed_frame_id': LaunchConfiguration('fixed_frame_id'),
            'child_frame_id': 'odom'
        }],
        # remappings=[
        #     ('imu', '/imu0'),
        #     ('cam0_image', '/cam0/image_raw')
        # ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='visualization',
        output='log',
        arguments=['-d', os.path.join(pkg_dir, 'rviz/vio_ros2.rviz')]
    )

    return LaunchDescription([
        fixed_frame_id_arg,
        vio_node,
        rviz_node
    ])