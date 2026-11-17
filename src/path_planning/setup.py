import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Azeem Ashraf',
    maintainer_email='azeemashraf9210@gmail.com',
    description='Custom A* Path Planner and B-Spline Smoother from MIL Framework',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'planner_node = path_planning.planner_node:main',
            'smoother_node = path_planning.smoother_node:main',
            'plot_path = path_planning.plot_path:main',
        ],
    },
)
