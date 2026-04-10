from setuptools import find_packages, setup

package_name = 'rover_xplore'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Philippe',
    maintainer_email='hady.azzy@epfl.ch',
    description='Rover XPlore - Navigation autonome et teleop',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = rover_xplore.camera_node:main',
            'mode_manager_node = rover_xplore.mode_manager_node:main',
        ],
    },
)
