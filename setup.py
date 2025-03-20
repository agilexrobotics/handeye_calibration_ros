from setuptools import setup
import os
import glob
from glob import glob


package_name = 'handeye_calibration_ros'

setup(
    name=package_name,
    version='0.0.0',  
    packages=[package_name], 
    install_requires=['setuptools', 'rclpy'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    zip_safe=True,
    author='agilex',
    author_email='agilex@todo.todo',
    description='TODO: Package description',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'handeye_calibration = handeye_calibration_ros.handeye_calibration:main',
        ],
    }
)
