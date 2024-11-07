from setuptools import find_packages, setup

package_name = 'standby_processing_manager'

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
    maintainer='root',
    maintainer_email='shuma.suzuki.2021@outlook.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rectangle_and_circle_marker_node = standby_processing_manager.rectangle_and_circle_marker_node:main',
            'move_rectangle_and_circle_marker_node = standby_processing_manager.move_rectangle_and_circle_marker_node:main',
            'localcost_and_move_node = standby_processing_manager.localcost_and_move_node:main',
            'goal_and_move_node = standby_processing_manager.goal_and_move_node:main',
        ],
    },
)
