from setuptools import find_packages, setup

package_name = 'vision_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aaron Jacobs & Ryan Jacobs',
    maintainer_email='aaronj0315@gmail.com & jacobsrya@gmail.com',
    description='ZED 2i blue-tape lane detection node + SLAM + visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_mapper = vision_perception.line_mapper:main',
            'lane_slam = vision_perception.lane_slam:main',
            'slam_visualizer = vision_perception.slam_visualizer:main',
        ],
    },
)
