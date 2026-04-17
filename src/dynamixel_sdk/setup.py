from setuptools import setup

package_name = 'dynamixel_sdk'

setup(
    name=package_name,
    version='3.7.51',
    packages=[package_name],
    package_dir={package_name: 'src/dynamixel_sdk'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Will Son',
    maintainer_email='willson@robotis.com',
    description='ROBOTIS Dynamixel SDK Python wrapper for ROS 2 (bundled v3.7.51)',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'hand_handle_node = seed_robotics.hand_handle_node:main',
        ],
    },
)
