from setuptools import setup

package_name = 'dog_description'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hzname',
    maintainer_email='hzname@example.com',
    description='URDF generator for the robot dog (geometry from dog_bringup/config/robot.yaml).',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['generate_urdf = dog_description.urdf:main']},
)
