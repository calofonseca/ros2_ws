from setuptools import find_packages, setup

package_name = 'IR_assignment1_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation_launch.py']),  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tiago',
    maintainer_email='tiago@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wall_follower = my_robot_controller.wall_follower:main"
        ],
    },
)
