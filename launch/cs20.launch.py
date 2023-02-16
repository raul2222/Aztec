

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            node_name='cs20_composition',
            package='cs20_ros',
            node_executable='cs20_composition',
            output='screen',
            parameters=[{
                'min_distance_range': 0,
                'min_distance_range': 2500,  

                'frame_id': 'face_link',
                'depth_image_filter': true,
                'exposure': 5000,
            }],
        ),
    ])