from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f1tenth_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),

        # Top-level config YAMLs (vesc_params, etc.)
        # Uses explicit list to avoid accidentally picking up directories.
        (os.path.join('share', package_name, 'config'),
            [f for f in glob(os.path.join('config', '*.yaml'))
             if os.path.isfile(f)]),

        # ZED config overrides (config/zed_config/*.yaml).
        # These are installed for reference; the actual runtime mount is via
        # the docker-compose volume:
        #   ../../src/f1tenth_bringup/config/zed_config:/opt/f1tenth_deps/...
        # so changes to these files take effect on the NEXT docker compose up,
        # no rebuild needed.
        (os.path.join('share', package_name, 'config', 'zed_config'),
            [f for f in glob(os.path.join('config', 'zed_config', '*.yaml'))
             if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Launch and config for the F1Tenth stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
