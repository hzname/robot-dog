from glob import glob

from setuptools import setup

package_name = 'dog_web'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hzname',
    maintainer_email='hzname@example.com',
    description='Web teleop page (virtual joystick, keyboard, gamepad) for the robot dog.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'web_teleop = dog_web.web_teleop:main',
        ],
    },
)
