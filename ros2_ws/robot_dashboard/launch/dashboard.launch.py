import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    cuda_libs = ':'.join([
        '/usr/local/cuda-12.6/targets/aarch64-linux/lib',
        '/usr/lib/aarch64-linux-gnu/nvidia',
        '/usr/lib/aarch64-linux-gnu/tegra',
    ])
    existing = os.environ.get('LD_LIBRARY_PATH', '')
    full_lib_path = cuda_libs + (':' + existing if existing else '')

    web_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), 'web')

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', full_lib_path),

        Node(package='yolo_detector', executable='detector_node',
             name='yolo_detector',
             parameters=[{'engine_path': '/home/justin/yolo26n.engine',
                          'confidence_threshold': 0.5, 'publish_image': True}],
             output='screen'),

        Node(package='state_machine', executable='state_machine_node',
             name='state_machine', output='screen'),

        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge',
             parameters=[{'port': 9090, 'address': '0.0.0.0'}],
             output='screen'),

        Node(package='web_video_server', executable='web_video_server',
             name='web_video_server',
             parameters=[{'port': 8080, 'address': '0.0.0.0'}],
             output='screen'),

        Node(package='robot_dashboard', executable='dashboard_server',
             name='dashboard_server',
             parameters=[{'web_dir': web_dir, 'port': 8888}],
             output='screen'),

        Node(package='drive_node', executable='drive_node',
             name='drive_node', output='screen'),

        Node(package='robot_dashboard', executable='system_monitor',
             name='system_monitor', output='screen'),
    ])
