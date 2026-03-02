from setuptools import find_packages, setup

package_name = 'dog_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/servo_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/servo_driver.launch.py']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='Robot Developer',
    maintainer_email='robot@example.com',
    description='Hardware interface for quadruped robot servo control via PCA9685',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver_node = dog_hardware.servo_driver_node:main',
        ],
    },
)
