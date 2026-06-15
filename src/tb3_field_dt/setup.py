from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'tb3_field_dt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'randomHumiditySensor = tb3_field_dt.RandomHumiditySensorNode:main',
            'humiditySensorInterpreter = tb3_field_dt.HumiditySensorInterpreterNode:main',
            'humidityMap = tb3_field_dt.HumidityMapNode:main',
            'irrigationRoutePlanner = tb3_field_dt.IrrigationRoutePlannerNode:main',
            'simpleRouteFollower = tb3_field_dt.SimpleRouteFollowerNode:main',
            'fieldHumiditySensor = tb3_field_dt.FieldHumiditySensorNode:main',
            'Nav2IrrigationRouteFollower = tb3_field_dt.Nav2IrrigationRouteFollowerNode:main',
            'irrigationController = tb3_field_dt.IrrigationControllerNode:main'
        ],
    },
)
