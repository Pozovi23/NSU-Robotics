from setuptools import find_packages, setup

package_name = 'robot_movement_depth_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/autonomous.launch.py']),
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
            'obstacle_avoider = robot_movement_depth_camera.obstacle_avoider:main',
        ],
    },
)
