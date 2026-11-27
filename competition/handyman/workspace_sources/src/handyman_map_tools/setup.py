from setuptools import find_packages, setup


package_name = 'handyman_map_tools'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup Team',
    maintainer_email='maintainer@example.com',
    description=(
        'Validate and visualize semantic maps exported from Handyman Unity '
        'scenes.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'import_unity_maps = '
            'handyman_map_tools.import_unity_maps:main',
            'validate_semantic_map = '
            'handyman_map_tools.validate_semantic_map:main',
            'semantic_marker_node = '
            'handyman_map_tools.semantic_marker_node:main',
        ],
    },
)
