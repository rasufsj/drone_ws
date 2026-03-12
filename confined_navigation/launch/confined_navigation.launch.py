from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # A-LOAM (Ouster LiDAR)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'aloam', 'aloam_ouster_os1.launch.py'],
            output='screen'
        ),

        # OctoMap Server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            parameters=[{
                'resolution': 0.1,
                'frame_id': 'uav1/ouster',
                'sensor_model/max_range': 30.0,
                'latch': True
            }],
            remappings=[('/cloud_in', '/uav1/ouster/point_cloud')]
        ),

        # Seus nós
        Node(package='confined_navigation', executable='takeoff_node', name='takeoff_node'),
        Node(package='confined_navigation', executable='perception_node', name='perception'),
        Node(package='confined_navigation', executable='planner_node', name='planner'),
        Node(package='confined_navigation', executable='navigation_node', name='navigation'),
    ])