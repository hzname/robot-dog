from glob import glob

from setuptools import setup

package_name = 'dog_gazebo'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hzname',
    maintainer_email='hzname@example.com',
    description='Gazebo simulation of the robot dog.',
    license='MIT',
    entry_points={'console_scripts': [
        'joint_command_bridge = dog_gazebo.joint_command_bridge:main',
        'walk_check = dog_gazebo.walk_check:main',
        'terrain_sweep = dog_gazebo.terrain_sweep:main',
    ]},
)
