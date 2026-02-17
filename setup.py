import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'f1tenth_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KoKoLates',
    maintainer_email='the21515@gmail.com',
    description='Bridge node for f1tenth simulation environment',
    license='MIT',
    entry_points={
        'console_scripts': [
            "bridge = f1tenth_simulator.bridge:main",
            "dummy = f1tenth_simulator.dummy:main"
        ],
    },
)
