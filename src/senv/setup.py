from setuptools import find_packages, setup

package_name = 'senv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_senv.py'])
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
            'driving = senv.driving:main',
            'camera = senv.camera:main',
        ],
    },
)
