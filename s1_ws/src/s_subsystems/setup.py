from setuptools import find_packages, setup

package_name = 's_subsystems'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aju3cob',
    maintainer_email='G.RajSuriyan@in.bosch.com',
    description='Mock subsystems (S2 perception, S3 motion, S4 actuation) for S1 robot simulation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            's2_perception = s_subsystems.s2_perception:main',
            's3_motion = s_subsystems.s3_motion:main',
            's4_actuation = s_subsystems.s4_actuation:main',
        ],
    },
)
