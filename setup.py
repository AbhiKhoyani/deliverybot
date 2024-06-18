import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'deliverybot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.sdf'))),
        (os.path.join('share', package_name, 'models', 'deliverybot'), glob(os.path.join('models', 'deliverybot' ,'*'))),
        (os.path.join('share', package_name, 'models', 'checkerboard'), glob(os.path.join('models', 'checkerboard' ,'*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.sh'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'cfg'), glob(os.path.join('cfg', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[pgm][yaml]'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='abhikhoyani@yahoo.in',
    description='deliveryBot for DFL.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mergepcd.py=deliverybot.mergepcd:main',
        ],
    },
)
