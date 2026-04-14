from setuptools import setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elyssa',
    maintainer_email='elyssa@robot.com',
    description='Vision — détection ArUco + caméra',
    license='MIT',
    tests_require=['pytest'],
    # Note : PostInstallCommand supprimé — ament_python gère les symlinks automatiquement
    entry_points={
        'console_scripts': [
            'aruco_detector_node = vision.aruco_detector_node:main',
            'box_detector_node = vision.box_detector_node:main',
        ],
    },
)