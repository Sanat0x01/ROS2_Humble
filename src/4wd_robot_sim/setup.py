from setuptools import setup
import os
from glob import glob

package_name = '4wd_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'obstacle_stop',
        'wall_avoidance',

    ],
    package_dir={'': '.'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanat',
    maintainer_email='ec24b1069@iiitdm.ac.in',
    description='4WD robot simulation with camera, LIDAR, and obstacle stop logic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_stop = obstacle_stop.obstacle_stop:main',
            'wall_avoidance = wall_avoidance.wall_avoidance:main',
            # Do not include orange_detector here
        ],
    },
)

