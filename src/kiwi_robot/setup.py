from setuptools import find_packages, setup
from setuptools.command.install import install
import os
from glob import glob

package_name = 'kiwi_robot'


class CustomInstall(install):
    """Custom install command that creates symlinks for ROS2 launch compatibility."""
    def run(self):
        # Run the standard install
        install.run(self)
        
        # Create symlinks from lib/package_name to bin/ for ROS2 launch system
        if self.root is None:
            install_dir = self.install_scripts
        else:
            install_dir = os.path.join(self.root, self.install_scripts.lstrip('/'))
        
        # Find the base install directory (remove /bin if present)
        if install_dir.endswith('/bin'):
            base_dir = install_dir[:-4]
        else:
            base_dir = os.path.dirname(os.path.dirname(install_dir))
        
        libexec_dir = os.path.join(base_dir, 'lib', package_name)
        bin_dir = os.path.join(base_dir, 'bin')
        
        # Ensure libexec directory exists
        os.makedirs(libexec_dir, exist_ok=True)
        
        # Create symlinks for each executable
        executables = [
            'so101_puppeteer',
            'so101_puppet',
            'kiwi_base',
            'kiwi_mobile_puppet',
            'kiwi_cam',
            'so101_calibrate',
            'so101_test_node'
        ]
        
        for exe in executables:
            bin_path = os.path.join(bin_dir, exe)
            libexec_path = os.path.join(libexec_dir, exe)
            
            # Remove existing file/symlink if it exists
            if os.path.exists(libexec_path) or os.path.islink(libexec_path):
                os.remove(libexec_path)
            
            # Create symlink from libexec to bin
            if os.path.exists(bin_path):
                os.symlink(os.path.relpath(bin_path, libexec_dir), libexec_path)

setup(
    name=package_name,
    version='0.0.0',
    cmdclass={'install': CustomInstall},
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Create lib/package_name directory for ROS2 launch system compatibility
        # Install a placeholder file to ensure the directory exists
        (os.path.join('lib', package_name), glob('resource/.gitkeep') if glob('resource/.gitkeep') else []),
    ],
    install_requires=['setuptools', 'pyserial', 'opencv-python-headless', 'numpy', 'feetech-servo-sdk'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Clean ROS2 implementation for SO101 arm and Kiwi base robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'so101_puppeteer = kiwi_robot.so101_puppeteer:main',
            'so101_puppet = kiwi_robot.so101_puppet:main',
            'kiwi_base = kiwi_robot.kiwi_base:main',
            'kiwi_mobile_puppet = kiwi_robot.kiwi_mobile_puppet:main',
            'kiwi_cam = kiwi_robot.kiwi_cam:main',
            'so101_calibrate = kiwi_robot.so101_calibrate:main',
            'so101_test_node = kiwi_robot.so101_test_node:main',
        ],
    },
)
