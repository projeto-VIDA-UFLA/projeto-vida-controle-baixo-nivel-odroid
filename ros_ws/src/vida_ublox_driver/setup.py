from glob import glob
from setuptools import find_packages, setup


package_name = 'vida_ublox_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Whilker',
    maintainer_email='whilker@example.com',
    description='Driver ROS 2 para GPS e IMU do u-blox NEO-M8U.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ublox_m8u_node = vida_ublox_driver.node:main',
        ],
    },
)

