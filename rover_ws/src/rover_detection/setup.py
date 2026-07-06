from setuptools import find_packages, setup
from glob import glob

package_name = 'rover_detection'

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
    description='Buried-threat sensor simulation, GPR/metal/VOC processing, fusion, evaluation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_sim_node = rover_detection.sensor_sim_node:main',
            'gpr_processor_node = rover_detection.gpr_processor_node:main',
            'fusion_node = rover_detection.fusion_node:main',
            'detection_eval_node = rover_detection.detection_eval_node:main',
        ],
    },
)
