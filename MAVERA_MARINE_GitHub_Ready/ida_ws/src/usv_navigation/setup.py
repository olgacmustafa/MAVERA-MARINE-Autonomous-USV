import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'usv_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join('share', package_name, 'models'),
            ['best.pt'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MAVERA MARINE',
    maintainer_email='maintainer@example.com',
    description='TEKNOFEST 2026 MAVERA MARINE USV autonomy package',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'los_node = usv_navigation.los_node:main',
            'obstacle_avoidance_node = usv_navigation.obstacle_avoidance_node:main',
            'pid_node = usv_navigation.pid_node:main',
            'data_logger_node = usv_navigation.data_logger_node:main',
            'estop = usv_navigation.estop:main',
            'haberlesme = usv_navigation.haberlesme:main',
        ],
    },
)
