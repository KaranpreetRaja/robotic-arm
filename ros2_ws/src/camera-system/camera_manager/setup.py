from setuptools import find_packages, setup

package_name = 'camera_manager'

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
    maintainer='karan',
    maintainer_email='karanpreet.singh.raja@gmail.com',
    description='This package is responsible for launching different camera streams and sending them to the client via WebRTC Protocol',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_manager_node = camera_manager.camera_manager_node:main",
        ],
    },
)
