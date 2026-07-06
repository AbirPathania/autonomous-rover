from setuptools import find_packages, setup
from glob import glob

package_name = 'rover_hw_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abipathania',
    maintainer_email='you@example.com',
    description='Jetson<->MCU framed link, bridge node, and SITL MCU (STM32/ODrive) emulator.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw_bridge_node = rover_hw_bridge.hw_bridge_node:main',
            'mcu_sim_node = rover_hw_bridge.mcu_sim_node:main',
        ],
    },
)
