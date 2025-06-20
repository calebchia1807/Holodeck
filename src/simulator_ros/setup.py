import os
import glob
from setuptools import find_packages, setup

package_name = 'simulator_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'unity_nav                  = simulator_ros.unity_nav:main',
            'unity_stand_crouch         = simulator_ros.unity_stand_crouch:main',
            'unity_rgb_camera           = simulator_ros.unity_rgb_camera:main',
            'unity_bgr_camera           = simulator_ros.unity_bgr_camera:main',
            'unity_depth_camera         = simulator_ros.unity_depth_camera:main',
            'unity_segmentation_camera  = simulator_ros.unity_segmentation_camera:main',
            'unity_bounding_box         = simulator_ros.unity_bounding_box:main',
        ],
    },
)
