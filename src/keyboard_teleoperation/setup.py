from glob import glob
from setuptools import find_packages, setup

package_name = 'keyboard_teleoperation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/firmware', glob('firmware/esp32_keyboard_bridge/*.ino')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Keyboard teleoperation and ESP32 serial bridge for a differential-drive rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = keyboard_teleoperation.keyboard_teleop_node:main',
            'esp32_cmd_bridge = keyboard_teleoperation.esp32_bridge_node:main',
        ],
    },
)
