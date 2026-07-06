from setuptools import find_packages, setup
from glob import glob

package_name = 'rover_safety'

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
    description='Safety/fault manager: watchdog, dropout/stall detection, e-stop, safe states.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_manager_node = rover_safety.safety_manager_node:main',
        ],
    },
)
