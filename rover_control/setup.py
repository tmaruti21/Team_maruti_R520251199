from setuptools import setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS2 Python nodes for rover control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_pwm_controller = rover_control.serial_pwm_controller:main',
            'keyboard_teleop = rover_control.keyboard_teleop:main',
            'lane_follower = rover_control.lane_follower:main',
            'camera_publisher = rover_control.camera_publisher:main',
            'hsv_calibrator = rover_control.hsv_calibrator:main',
            'hsv_calculator = rover_control.hsv_calculator:main',
        ],
    },
)