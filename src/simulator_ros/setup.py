from setuptools import setup

package_name = 'simulator_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for AI2-THOR control',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unity_agent_controller = simulator_ros.unity_agent_controller:main',
            'unity_agent_publisher  = simulator_ros.unity_agent_publisher:main',
        ],
    },
)
