from setuptools import setup
import os
from glob import glob

package_name = 'rufus_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Will Bullen',
    maintainer_email='will@rufus-robot.com',
    description='Hardware drivers for RUFUS robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ddsm315_driver = rufus_drivers.ddsm315_driver:main',
            'st3215_driver = rufus_drivers.st3215_driver:main',
        ],
    },
)

