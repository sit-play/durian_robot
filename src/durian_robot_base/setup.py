from setuptools import find_packages, setup

package_name = 'durian_robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='play',
    maintainer_email='sittichok3051@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        'motor_controller = durian_robot_base.motor_controller:main',
            'camera_controller = durian_robot_base.camera_controller:main',
            'keyboard_teleop = durian_robot_base.keyboard_teleop:main',
        ],
    },
)
