from setuptools import setup
import os
from glob import glob

package_name = 'composer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "log"), glob("log/*.log")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deniz',
    maintainer_email='deniz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'muto_composer = composer.muto_composer:main',
            'compose_plugin = composer.compose_plugin:main',
            'launch_plugin = composer.launch_plugin:main',
            'introspector = composer.introspector:main',
            'log_publisher = composer.log_publisher:main'
        ],
    },
)
