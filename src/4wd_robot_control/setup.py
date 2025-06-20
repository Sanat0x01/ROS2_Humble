from setuptools import find_packages, setup

package_name = '4wd_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanat',
    maintainer_email='ec24b1069@iiitdm.ac.in',
    description='A ROS 2 Python package to stop the 4WD robot when an obstacle is detected using LIDAR data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stop_on_obstacle = 4wd_robot_control.stop_on_obstacle:main',
        ],
    },
)

