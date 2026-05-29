from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_dashboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacy',
    maintainer_email='ignacy@todo.todo',
    description='Web dashboard for tb3 field robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard_bridge = tb3_dashboard.dashboard_bridge:main',
            'dummy_publisher  = tb3_dashboard.dummy_publisher:main',
        ],
    },
)
