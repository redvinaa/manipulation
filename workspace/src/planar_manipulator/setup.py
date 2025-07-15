from setuptools import setup

package_name = 'planar_manipulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='redvinaa@gmail.com',
    description='Excercises with a planar 2-link manipulator',
    license='All rights reserved',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'planar_manipulator = planar_manipulator.planar_manipulator:main',
            'grasp_pose = planar_manipulator.grasp_pose:main',
            'planar_manipulator_velocity_control = '
            'planar_manipulator.planar_manipulator_velocity_control:main',
        ],
    },
)
