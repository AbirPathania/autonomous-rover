from setuptools import find_packages, setup
from glob import glob

package_name = 'rover_localization'

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
    description='State estimation: dual-EKF, FAST-LIO2 integration, degraded dead-reckoning manager.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_mode_manager = rover_localization.localization_mode_manager:main',
        ],
    },
)
