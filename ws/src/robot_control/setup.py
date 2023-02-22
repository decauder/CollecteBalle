from setuptools import setup, find_namespace_packages

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(include=['robot_control.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='donuts',
    maintainer_email='danut462@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick = robot_control.joystick:main',
            'robot_position_controller = robot_control.robot_position_controller:main'
        ],
    },
)
