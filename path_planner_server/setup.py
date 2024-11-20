from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='markbilginer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_pose_action_client = path_planner_server.scripts.nav_to_pose_action_client:main',  # Updated path
        ],
    },
)
