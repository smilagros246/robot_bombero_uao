from setuptools import setup, find_packages


package_name = 'robot_bombero_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # detecta automáticamente todos los módulos
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'opencv-python'],
    zip_safe=True,
    maintainer='sofia',
    maintainer_email='sofia.castano@uao.edu.co',
    description='Nodo para visualización RGB y térmica del robot bombero.',
    license='Tu licencia',
    tests_require=['pytest'],
    package_data={
        '': ['launch/*.py', 'resource/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/ros2_packages', ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/raspberry_camera_launch.py', 'launch/vision_launch.py']),
    ],
    entry_points={
        'console_scripts': [
            'vision_node = robot_bombero_vision.vision_node:main',
            'raspberry_camera_node = robot_bombero_vision.raspberry_camera_node:main'
        ],
    },
)


