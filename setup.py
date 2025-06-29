import os
from glob import glob
from setuptools import setup

package_name = 'webots_turtlebot4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'protos'), glob(os.path.join('protos', 'Turtlebot4.proto'))),
        (os.path.join('share', package_name, 'protos'), glob(os.path.join('protos', '**/*'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', 'turtlebot_webots.urdf'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', 'ros2control.yml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonas Kuckling',
    maintainer_email='jonas.kuckling@uni-konstanz.de',
    description='Webots simulation for the TurtleBot4',
    license='This package is available under the MIT license.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_intensity_publisher = webots_turtlebot4.ir_intensity_publisher:main'
        ],
    },
)
