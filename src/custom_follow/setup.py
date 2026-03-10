from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'custom_follow'

# Include model file in share directory if it has been trained already
_model_files = glob('model/*.pt')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + (
        [('share/' + package_name + '/model', _model_files)]
        if _model_files else []
    ),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='CNN-based lane classification and autonomous navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'infer_node = custom_follow.infer_node:main',
        ],
    },
)
