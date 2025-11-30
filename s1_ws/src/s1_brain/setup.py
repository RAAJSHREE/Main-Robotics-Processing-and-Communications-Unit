from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's1_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aju3cob',
    maintainer_email='G.RajSuriyan@in.bosch.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heartbeat_bridge = s1_brain.heartbeat_bridge:main',
            'logs_bridge = s1_brain.logs_bridge:main',
            'telemetry_bridge = s1_brain.telemetry_bridge:main',
            'telemetry_publisher = s1_brain.telemetry_publisher:main',
            'diagnostics_node = s1_brain.diagnostics_node:main',
            'master_controller = s1_brain.master_controller:main',
            'command_test = s1_brain.command_test:main',
            'command_bridge = s1_brain.command_bridge:main',
        ],
    },
)
