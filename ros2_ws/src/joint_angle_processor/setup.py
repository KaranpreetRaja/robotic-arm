from setuptools import find_packages, setup

package_name = 'joint_angle_processor'

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
    maintainer='abdi',
    maintainer_email='abdirazak140@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joint_angle_processor = joint_angle_processor.joint_angle_processor:main",
            "joint_angle_stringify = joint_angle_processor.joint_angle_stringify:main"
        ],
    },
)
