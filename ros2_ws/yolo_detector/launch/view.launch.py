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

    # Prepend CUDA paths — don't replace the existing LD_LIBRARY_PATH
    existing = os.environ.get('LD_LIBRARY_PATH', '')
    full_lib_path = cuda_libs + (':' + existing if existing else '')

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', full_lib_path),

        Node(
            package='yolo_detector',
            executable='detector_node',
            name='yolo_detector',
            parameters=[{
                'engine_path': '/home/justin/yolo26n.engine',
                'confidence_threshold': 0.5,
                'publish_image': True,
            }],
            output='screen',
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            arguments=['/camera/image_raw'],
            output='screen',
        ),
    ])
