from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'pose_estimator_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch_config.yaml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='LucyCVieyra',
    maintainer_email='9496@soyunaq.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            "px4_driver = pose_estimator_pkg.px4_driver:main",
            "move_drone = pose_estimator_pkg.move_drone:main",
            "take_photos = pose_estimator_pkg.take_photos:main",
            "vehicle_pose = pose_estimator_pkg.vehicle_pose:main",
            "save_pose_image = pose_estimator_pkg.save_pose_image:main",
            "fly_drone = pose_estimator_pkg.fly_drone:main",
            "save_pose_image_rpy = pose_estimator_pkg.save_pose_image_rpy:main",
            "waypoint_follower = pose_estimator_pkg.waypoint_follower:main",
            "vehicle_attitude = pose_estimator_pkg.vehicle_attitude:main",
            "nn_pose_estimator = pose_estimator_pkg.nn_pose_estimator:main",
            "pose_pub = pose_estimator_pkg.pose_pub:main",
            "pose_error = pose_estimator_pkg.pose_error:main",
        ],
    },
)