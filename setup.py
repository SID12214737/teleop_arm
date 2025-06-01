from setuptools import find_packages, setup

package_name = 'teleop_arm'

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
    maintainer='miracle',
    maintainer_email='uzmir6358@gmail.com',
    description='Custom joystick teleop for TurtleBot3 manipulator arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_arm_node = teleop_arm.teleop_arm:main'
        ],
    },
)
