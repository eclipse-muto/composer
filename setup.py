#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
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
            "provision_plugin = composer.plugins.provision_plugin:main",
            "launch_plugin = composer.plugins.launch_plugin:main",
            "daemon = composer.introspection.muto_daemon:main",
        ],
    },
)
