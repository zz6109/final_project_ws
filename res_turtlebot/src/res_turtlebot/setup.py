from setuptools import find_packages, setup

package_name = 'res_turtlebot'

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
    maintainer='test',
    maintainer_email='cjp8042000@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "respeaker_turtlebot_test.py = res_turtlebot.respeaker_turtlebot_test.py:main",
        "respeaker_turtlebot = res_turtlebot.respeaker_turtlebot:main",
        "tuning = res_turtlebot.tuning:main",
        ],
    },
)
