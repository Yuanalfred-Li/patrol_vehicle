from setuptools import find_packages, setup

package_name = 'patrol_auto_command_mux'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@example.com',
    description='Automatic command multiplexer for patrol vehicle',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'auto_command_mux_node = '
            'patrol_auto_command_mux.auto_command_mux_node:main',
        ],
    },
)
