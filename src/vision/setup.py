from setuptools import setup, find_packages
from setuptools.command.install import install
import os

package_name = 'vision'

class PostInstallCommand(install):
    """Commande post-installation pour créer les liens symboliques"""
    def run(self):
        install.run(self)
        # Crée le lien symbolique dans lib/vision/
        lib_dir = os.path.join(self.install_lib, '..', '..', 'lib', package_name)
        bin_dir = os.path.join(self.install_lib, '..', '..', 'bin')
        
        os.makedirs(lib_dir, exist_ok=True)
        
        # Liste des executables à lier
        executables = ['aruco_detector_node']
        
        for script in executables:
            src = os.path.join(bin_dir, script)
            dst = os.path.join(lib_dir, script)
            if os.path.exists(src) and not os.path.exists(dst):
                os.symlink(os.path.relpath(src, lib_dir), dst)
                print(f"Created symlink: {dst} -> {src}")

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/aruco_detection.launch.py']),  # <-- LIGNE AJOUTÉE
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elyssa',
    maintainer_email='elyssa@robot.com',
    description='Package de vision pour détection ArUco',
    license='MIT',
    tests_require=['pytest'],
    cmdclass={
        'install': PostInstallCommand,
    },
    entry_points={
        'console_scripts': [
            'aruco_detector_node = vision.aruco_detector_node:main',
        ],
    },
)