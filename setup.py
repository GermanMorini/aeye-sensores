from setuptools import find_packages, setup

package_name = 'sensores'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['pixhawk_dashboard.html']),
        ('share/' + package_name + '/launch', [
            'launch/pixhawk.launch.py',
            'launch/rs16.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/rs16.yaml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    description='Pixhawk reader node that publishes ROS 2 topics via MAVLink',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixhawk_driver = sensores.pixhawk_driver:main',
            'sensores_web = sensores.web_server:main',
        ],
    },
)
