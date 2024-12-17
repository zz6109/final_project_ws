from setuptools import setup
import os
from glob import glob

package_name = 'custom_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autocar',
    maintainer_email='autocar@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_angle_pub = custom_bringup.sound_angle_pub:main',
            'sound_angle_sub = custom_bringup.sound_angle_sub:main',
            'arm_service_server = custom_bringup.arm_service_server:main',
        ],
    },
)
