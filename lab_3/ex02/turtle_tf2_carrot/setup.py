from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_tf2_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gleb',
    maintainer_email='gleb.zhigalov.2005@mail.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamic_frame_tf2_broadcaster = turtle_tf2_carrot.dynamic_frame_tf2_broadcaster:main',
            'turtle_tf2_broadcaster = turtle_tf2_carrot.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = turtle_tf2_carrot.turtle_tf2_listener:main',
        ],
    },
)
