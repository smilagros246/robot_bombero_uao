from setuptools import setup

package_name = 'robot_bombero_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sofia',
    maintainer_email='sofia@todo.todo',
    description='Nodo de teleoperación por joystick para el robot bombero',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = robot_bombero_teleop.teleop_node:main',
        ],
    },
)
