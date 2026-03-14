from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Autonomous navigation hardware bridge with cmd_vel multiplexing for auto and keyboard control.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_mux = auto_nav.cmd_vel_mux_node:main',
            'esp32_auto_bridge = auto_nav.esp32_bridge_node:main',
        ],
    },
)
