from setuptools import find_packages, setup

package_name = 'task_coordinator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/arm_poses.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Febin TJ',
    maintainer_email='febintj007@gmail.com',
    description='Task coordination node for autonomous home service robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_coordinator_node = task_coordinator.task_coordinator_node:main',
            'bring_food = task_coordinator.bring_food_client:main',
            'pose_recorder = task_coordinator.pose_recorder:main',
            'food_detector = task_coordinator.food_detector:main',
        ],
    },
)
