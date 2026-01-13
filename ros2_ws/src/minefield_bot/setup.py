from setuptools import setup
import os
from glob import glob

package_name = 'minefield_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Minefield Robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_bridge = minefield_bot.hardware_bridge:main',
            'mapper = minefield_bot.simple_mapper:main',
            'planner = minefield_bot.astar_planner:main',
        ],
    },
)
