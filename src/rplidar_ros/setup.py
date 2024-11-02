from setuptools import setup

package_name = 'rplidar_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['igvc_slam_node'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'igvc_slam_node = igvc_slam_node:main',  # Update this if your main function is different
            'imu = imu:main',
        ],
    },
)
