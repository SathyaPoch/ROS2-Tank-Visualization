from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tank_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mnikaa',
    maintainer_email='mnikaa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tank_body_node = tank_visualizer.tank_body_node:main',
            'turret_controller = tank_visualizer.turret_controller:main',
            'keyboard_teleop    = tank_visualizer.keyboard_teleop:main',  # ← ADD THIS
        ],
    },
)
