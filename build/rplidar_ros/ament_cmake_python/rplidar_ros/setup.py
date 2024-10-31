from setuptools import find_packages
from setuptools import setup

setup(
    name='rplidar_ros',
    version='2.1.4',
    packages=find_packages(
        include=('rplidar_ros', 'rplidar_ros.*')),
)
