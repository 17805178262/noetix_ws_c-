from setuptools import setup
from glob import glob

package_name = 'noetix_mujoco'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noetix Robotics',
    maintainer_email='robotics@noetix.ai',
    description='MuJoCo backend with real Noetix ROS 2 actuator semantics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mujoco_backend = noetix_mujoco.backend:main',
        ],
    },
)
