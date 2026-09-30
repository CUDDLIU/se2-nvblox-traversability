from setuptools import setup
from glob import glob

package_name = 'se2_nvblox_surface_shadow'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'reference_map = se2_nvblox_surface_shadow.reference_map:main',
        'crop_cloud = se2_nvblox_surface_shadow.crop_cloud:main',
        'surface_adapter = se2_nvblox_surface_shadow.surface_adapter:main',
        'cloud_audit = se2_nvblox_surface_shadow.cloud_audit:main',
        'odom_pose_adapter = se2_nvblox_surface_shadow.odom_pose_adapter:main',
    ]},
)
