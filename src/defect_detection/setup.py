from glob import glob
import os

from setuptools import find_namespace_packages, setup

package_name = 'defect_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_namespace_packages(
        include=[package_name, package_name + '.*'],
        exclude=['test'],
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avaradar',
    maintainer_email='arunvaradarajan3@gmail.com',
    description='ROS 2 camera, detection, and LiDAR fusion nodes.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_publisher = defect_detection.spot_cam_loading.image_publisher:main',
            'yolo_detector = defect_detection.defect_detection.yolo_detector:main',
            'visualization_node = '
            'defect_detection.defect_detection.visualization_node:main',
            'trimble_scan_watcher = '
            'defect_detection.digital_twin.trimble_scan_watcher:main',
            'pointcloud_to_occupancy = '
            'defect_detection.digital_twin.pointcloud_to_occupancy:main',
            'defect_map_node = '
            'defect_detection.digital_twin.defect_map_node:main',
            'scan_decision_node = '
            'defect_detection.digital_twin.scan_decision_node:main',
            'trimble_windows_bridge = '
            'defect_detection.digital_twin.trimble_windows_bridge:main',
            'frame_anchor_node = '
            'defect_detection.digital_twin.frame_anchor_node:main',
            'infrastructure_planner = '
            'defect_detection.digital_twin.infrastructure_planner:main',
            'robot_goal_bridge = '
            'defect_detection.digital_twin.robot_goal_bridge:main',
            'depth_fusion_node = '
            'defect_detection.digital_twin.depth_fusion_node:main',
            'depth_localization_bridge = '
            'defect_detection.digital_twin.depth_localization_bridge:main',
            'spot_localization_bridge = '
            'defect_detection.digital_twin.spot_localization_bridge:main',
            'eap_lidar_bridge = '
            'defect_detection.digital_twin.eap_lidar_bridge:main',
            'mission_manager = '
            'defect_detection.digital_twin.mission_manager:main',
            'synthetic_field_node = '
            'defect_detection.simulation.synthetic_field_node:main',
            'sim_detection_node = '
            'defect_detection.simulation.sim_detection_node:main',
            'sim_scan_node = '
            'defect_detection.simulation.sim_scan_node:main',
            'goal_driver_node = '
            'defect_detection.simulation.goal_driver_node:main',
            'virtual_site_node = '
            'defect_detection.simulation.virtual_site_node:main',
        ],
    },
)
