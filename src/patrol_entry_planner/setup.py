from setuptools import find_packages, setup

package_name = 'patrol_entry_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@example.com',
    description='Hybrid A star entry planner for patrol vehicle',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'entry_planner_node = '
            'patrol_entry_planner.entry_planner_node:main',
        ],
    },
)
