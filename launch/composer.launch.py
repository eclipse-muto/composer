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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    muto_namespace_arg = DeclareLaunchArgument("muto_namespace", default_value="muto")

    composer_params = os.path.join(
        get_package_share_directory("composer"),
        "config",
        "composer.yaml"
    )

    node_composer = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="muto_composer",
        output="screen",
        parameters=[
            composer_params,
        ],
    )

    node_compose_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="compose_plugin",
        output="screen",
        parameters=[
            composer_params,
        ],
    )

    node_native_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="native_plugin",
        output="screen",
        parameters=[
            composer_params,
        ],
    )

    node_launch_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="launch_plugin",
        output="screen",
        parameters=[
            composer_params,
        ],
    )

    # Launch Description Object
    ld = LaunchDescription()

    ld.add_action(muto_namespace_arg)
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_native_plugin)
    ld.add_action(node_launch_plugin)
    return ld
