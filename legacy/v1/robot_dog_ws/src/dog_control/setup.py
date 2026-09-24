from setuptools import setup

package_name = 'dog_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/balance_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/balance_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Control algorithms for the robot dog gait',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_controller = dog_control.gait_controller:main',
            'test_gait = dog_control.test_gait:main',
            'balance_controller = dog_control.balance_controller:main',
        ],
    },
)
