from setuptools import find_packages, setup
from glob import glob

package_name = 'rover_navigation'

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
    description='Nav2 navigation over the terrain costmap with threat keepout zones.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'threat_zone_node = rover_navigation.threat_zone_node:main',
        ],
    },
)
