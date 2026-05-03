from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'warehouse_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iasl',
    maintainer_email='iasl@example.com',
    description='Warehouse offboard mission package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_point = warehouse_offboard.goto_point:main',
            'chat_mission_ui = warehouse_offboard.chat_mission_ui:main',
            'gz_camera_bridge = warehouse_offboard.gz_camera_bridge:main',
            'aruco_land = warehouse_offboard.aruco_land:main',
            'inventory_vision_shelf = warehouse_offboard.inventory_vision_shelf:main',
            # 팀원 추가 노드
            'llm_node = warehouse_offboard.llm_node:main',
            'mission_sequencer = warehouse_offboard.mission_sequencer:main',
            'llm_scan_analyzer = warehouse_offboard.llm_scan_analyzer:main',
            'inventory_reporter = warehouse_offboard.inventory_reporter:main',
            'result_report_node = warehouse_offboard.result_report_node:main',
            'inv_counter_node = warehouse_offboard.inv_counter_node:main',
            'qr_detection_node = warehouse_offboard.qr_detection_node:main',
        ],
    },
)
