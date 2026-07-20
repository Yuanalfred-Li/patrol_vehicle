from setuptools import find_packages, setup

package_name = 'patrol_route_follower'

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
    description='Route following for the patrol vehicle',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_follower_node = '
            'patrol_route_follower.route_follower_node:main',
        ],
    },
)
