from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pose_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elias',
    maintainer_email='93720605+EliasTDam@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "pose_commander = pose_commander.pose_commander:main",
        "pose_publisher = pose_commander.pose_publisher:main",
        "target_joints_publisher = pose_commander.target_joints_publisher:main",
        ],
    },
)
