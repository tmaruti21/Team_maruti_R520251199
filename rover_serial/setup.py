from setuptools import find_packages, setup

package_name = 'rover_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='team.maruti21@gmail.com',
    description='Serial bridge: ESP32 IMU + encoders -> ROS2 Imu + Odometry.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "esp_serial = rover_serial.esp_serial:main",
        ],
    },
)
