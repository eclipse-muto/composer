#
#  Copyright (c) 2025 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#

import os
from setuptools import find_packages, setup
from glob import glob

PACKAGE_NAME = "composer"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["docker", "setuptools", "jsonschema"],
    zip_safe=True,
    maintainer="composiv.ai",
    maintainer_email="info@composiv.ai",
    description="Composer provides orchestration utilites for ROS2 environment",
    license="Eclipse Public License",
    tests_require=["unittest", "pytest"],
    test_suite="test",
    entry_points={
        "console_scripts": [
            "muto_composer = composer.muto_composer:main",
            "compose_plugin = composer.plugins.compose_plugin:main",
            "native_plugin = composer.plugins.native_plugin:main",
            "launch_plugin = composer.plugins.launch_plugin:main",
            "daemon = composer.introspection.muto_daemon:main",
        ],
    },
)
