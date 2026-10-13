from glob import glob

from setuptools import find_packages, setup

package_name = 'chassis_communication'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Chassis Maintainer',
    maintainer_email='maintainer@example.com',
    description='Keyboard cmd_vel control and a two-wheel int32 UART bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_node = chassis_communication.keyboard_node:main',
            'serial_node = chassis_communication.serial_node:main',
        ],
    },
)
