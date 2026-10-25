import os
from glob import glob
from setuptools import setup

package_name = 'clinic_delivery'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models', 'delivery_bot'),
         glob('models/delivery_bot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PAI student',
    maintainer_email='noreply@example.com',
    description='Clinic delivery robot simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'go_to = clinic_delivery.go_to:main',
            'mission = clinic_delivery.mission:main',
            'plot_mission = clinic_delivery.plot_mission:main',
        ],
    },
)
