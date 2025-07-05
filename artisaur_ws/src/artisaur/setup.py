from setuptools import setup

package_name = 'artisaur'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['rospy', 'sensor_msgs', 'geometry_msgs', 'std_msgs', 'urdf_parser_py'],
    zip_safe=True,
    maintainer='Raffaello Bonghi',
    maintainer_email='raffaello@rnext.it',
    description='A ROS1 package for controlling the NanoSaur robot.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'artisaur_node = artisaur.artisaur_node:main',
        ],
    },
)