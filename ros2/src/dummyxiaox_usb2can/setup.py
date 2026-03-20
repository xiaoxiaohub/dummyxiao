import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dummyxiaox_usb2can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiehuang',
    maintainer_email='hydrangeahuman@gmail.com',
    description='USB serial bridge node for CtrlStep motor control via REF Core Board',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'usb2can_node = dummyxiaox_usb2can.usb2can_node:main',
        ],
    },
)
