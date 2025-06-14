from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_all_nodes.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanat',
    maintainer_email='ec24b1069@iiitdm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main","draw_circle = my_robot_controller.draw_circle:main",
            "pose_subscriber = my_robot_controller.pose_subscriber:main",
            "number_publisher = my_robot_controller.number_publisher:main",
            "number_subscriber = my_robot_controller.number_subscriber:main",

        ],
    },
)
