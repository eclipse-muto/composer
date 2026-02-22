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
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "muto_composer"

setup(
    name=PACKAGE_NAME,
    version="0.42.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "jsonschema"],
    zip_safe=True,
    maintainer="composiv.ai",
    maintainer_email="info@composiv.ai",
    description=(
        "Eclipse Muto Composer - stack deployment and orchestration engine"
        " that manages provisioning, launching, and lifecycle of ROS 2 software stacks"
    ),
    license="Eclipse Public License v2.0",
    python_requires=">=3.10",
    entry_points={
        "console_scripts": [
            "muto_composer = muto_composer.muto_composer:main",
            "compose_plugin = muto_composer.plugins.compose_plugin:main",
            "provision_plugin = muto_composer.plugins.provision_plugin:main",
            "launch_plugin = muto_composer.plugins.launch_plugin:main",
        ],
    },
)
