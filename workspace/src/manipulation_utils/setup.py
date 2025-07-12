from setuptools import setup

package_name = 'manipulation_utils'

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
    description='Util functions for the manipulation excercises',
    license='All rights reserved',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
        ],
    },
)
