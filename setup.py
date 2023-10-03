from setuptools import find_packages, setup

package_name = 'warmup_project'

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
    maintainer='cjhi',
    maintainer_email='chilty@olin.edu',
    description='Features for the CompRobo warmup project at Olin College,\
                comprobo23.github.io/',
    license='MIT',
    tests_require=['pytest'],
    entry_points={ #Has all the python files without the .py for colcon
        'console_scripts': [
            'angle_helpers = robot_localization.angle_helpers:main',
            'helper_functions = robot_localization.helper_functions:main',
            'occupancy_field = robot_localization.occupancy_field:main',
            'pf = robot_localization.pf:main',
            'robotControl = robot_localization.robotControl:main',
        ],
    },
)