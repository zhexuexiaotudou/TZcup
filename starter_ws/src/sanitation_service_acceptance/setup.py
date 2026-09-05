from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'sanitation_service_acceptance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanitation Vehicle Team',
    maintainer_email='team@example.com',
    description='Physical service-interface acceptance fixtures and collector.',
    license='Apache-2.0',
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'formal_service_acceptance_collector = sanitation_service_acceptance.collector:main',
        ]
    },
)
