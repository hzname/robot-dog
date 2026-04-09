from setuptools import setup, find_packages
from pathlib import Path

package_name = 'dog_web'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', [
            'templates/dashboard.html',
            'templates/control.html',
            'templates/state.html',
            'templates/calibration.html',
        ]),
        ('share/' + package_name + '/static', [
            'static/style.css',
            'static/common.js',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'web_server = dog_web.web_server:main',
        ],
    },
)
