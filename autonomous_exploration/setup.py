from setuptools import find_packages, setup

package_name = 'autonomous_exploration'

setup(
    name='autonomous-exploration', 
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Kusa',
    maintainer_email='evan.kusa@duke.edu',
    description='This package provides autonomous exploration for TurtleBot4 using ROS2 Humble.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_explorer = autonomous_exploration.autonomous_explorer:main',
        ],
    },
)
