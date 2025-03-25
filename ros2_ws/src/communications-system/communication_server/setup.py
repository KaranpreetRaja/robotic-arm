from setuptools import find_packages, setup

package_name = 'communication_server'

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
    description='This package runs a fast api app and allows for HTTP and websocket interfacing with ros2',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "communication_server = communication_server.communication_server:main",
        ],
    },
)
