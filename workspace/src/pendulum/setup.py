from setuptools import setup

package_name = 'pendulum'

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
    description='Simple pendulum model with a controller',
    license='All rights reserved',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pendulum_PID_control = pendulum.pendulum_PID_control:main',
            'pendulum_torque_control = pendulum.pendulum_torque_control:main',
        ],
    },
)
