from setuptools import find_packages, setup
import os
from glob import glob

PACKAGE_NAME = 'composer'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.yaml")),
    ],
    install_requires=['docker', 'setuptools'],
    zip_safe=True,
    maintainer='composiv.ai',
    maintainer_email='info@composiv.ai',
    description='Composer provides orchestration utilites for ROS2 environment',
    license='Eclipse Public License',
    tests_require=['unittest'],
    test_suite='test',
    entry_points={
        'console_scripts': [
            'muto_composer = composer.muto_composer:main',
            'compose_plugin = composer.compose_plugin:main',
            'containerize_plugin = composer.containerize_plugin:main',
            'provision_plugin = composer.provision_plugin:main',
            'native_plugin = composer.native_plugin:main',
            'launch_plugin = composer.launch_plugin:main',
            'introspection_plugin = composer.introspection_plugin:main'
        ],
    },
)
