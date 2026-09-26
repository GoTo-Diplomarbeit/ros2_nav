from setuptools import find_packages, setup

package_name = 'go2_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Peter Promok, Kilian Schneider, Micah Labendia, Alex Pticek',
    maintainer_email='peter.promok07@htl-salzburg.ac.at, kilian.schneider07@htl-salzburg.ac.at, micah.labendia07@htl-salzburg.ac.at, alex.pticek@htl-salzburg.ac.at',
    description='Go2 mapping and navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'movement_node = go2_nav.movement_node:main',
            'vision_node = go2_nav.vision_node:main',
            'planner_node = go2_nav.planner_node:main',
        ],
    },
)
