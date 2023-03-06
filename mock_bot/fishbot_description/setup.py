from setuptools import setup
from glob import glob
import os

package_name = 'fishbot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'world'), glob('world/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='setsuna',
    maintainer_email='136092213@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rotate_wheel= fishbot_description.rotate_wheel:main",
            "state_sync= fishbot_description.state_sync:main",
            "spawn_entity= fishbot_description.spawn_entity:main"
        ],
    },
)
