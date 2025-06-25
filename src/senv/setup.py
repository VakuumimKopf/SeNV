from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'senv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/launch_senv_lane.py']),
        ('share/' + package_name, ['package.xml', 'launch/launch_senv.py']),
        # Add launch files
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oliver',
    maintainer_email='oliverlorenz.de@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_con = senv.lane_con:main',
            'intersection_con = senv.intersection_con:main',
            'park_con = senv.park_con:main',
            'driver = senv.driver:main',
            'camera = senv.camera:main',
            'laserscanner = senv.laserscanner:main',
            'obstacle_con = senv.obstacle_con:main',
            'lane_detect=senv.lane_detect:main',
            'state_machine=senv.state_machine:main',
            'crosswalk_con=senv.crosswalk_con:main',
        ],
    },
)
