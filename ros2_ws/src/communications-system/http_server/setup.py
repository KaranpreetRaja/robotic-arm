from setuptools import find_packages, setup

package_name = 'http_server'

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
    maintainer='Karanpreet Raja',
    maintainer_email='86526687+KaranpreetRaja@users.noreply.github.com',
    description='Modular HTTP Communication Server that supports ros2 pub_sub and services',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "http_server = http_server.http_server:main",
        ],
    },
)
