import os
from glob import glob
from setuptools import setup

package_name = 'stella_wlkata_sm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dltlgos',
    maintainer_email='dltlgos@gmail.com',
    description='subscribe wlkata topic and send to shared memory',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stella_wlkata_node = stella_wlkata_sm.main:main',
        ],
    },
)
