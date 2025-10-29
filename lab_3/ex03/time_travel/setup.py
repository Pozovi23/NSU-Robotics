from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'time_travel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gleb',
    maintainer_email='gleb.zhigalov.2005@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = time_travel.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = time_travel.turtle_tf2_listener:main',
            'turtle_tf2_time_travel_listener = time_travel.turtle_tf2_time_travel_listener:main',
        ],
    },
)
