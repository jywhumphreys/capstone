import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'),    glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='justin',
    maintainer_email='justin@todo.todo',
    description='Web dashboard for robot monitoring and control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_server   = robot_dashboard.dashboard_server:main',
            'system_monitor     = robot_dashboard.system_monitor_node:main',
        ],
    },
)
