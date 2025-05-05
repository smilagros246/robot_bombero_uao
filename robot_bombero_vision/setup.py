from setuptools import setup

package_name = 'robot_bombero_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'opencv-python'],
    zip_safe=True,
    maintainer='sofia',
    maintainer_email='sofia.castano@uao.edu.co',
    description='Nodo para visualización RGB y térmica del robot bombero.',
    license='Tu licencia',
    tests_require=['pytest'],
    package_data={
        '': ['launch/*', 'resource/*', 'robot_bombero_vision/*'],  # Incluir todos los archivos necesarios
    },
    data_files=[
        ('share/ament_index/resource_index/ros2_package', ['package.xml']),  # Asegurar que package.xml se instale correctamente
    ],
    entry_points={
        'console_scripts': [
            'vision_node = robot_bombero_vision.vision_node:main'
        ],
    },
)

