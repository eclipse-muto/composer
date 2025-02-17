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


import sys
import os
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions

BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
RED = "\033[91m"


def resolve_substitutions(context, value):
    # If the value is a list or tuple of substitutions, we attempt to perform them
    if isinstance(value, list) or isinstance(value, tuple):
        try:
            return "".join(perform_substitutions(context, value))
        except:
            return str(value)
    # If it's not a list/tuple, just convert to string
    return str(value)


def recursively_extract_entities(
    entities, context, nodes, composable_nodes, containers
):
    for entity in entities:
        try:
            if isinstance(entity, IncludeLaunchDescription):
                included_ld = entity.launch_description_source.get_launch_description(
                    context=context
                )
                included_ld.visit(context)  # Expand includes, substitutions, etc.
                recursively_extract_entities(
                    included_ld.entities, context, nodes, composable_nodes, containers
                )
            elif isinstance(entity, GroupAction):
                sub_entities = entity.get_sub_entities()
                recursively_extract_entities(
                    sub_entities, context, nodes, composable_nodes, containers
                )
            else:
                if not already_found(entity, nodes, composable_nodes, containers):
                    if isinstance(entity, Node):
                        resolved_executable = resolve_substitutions(
                            context, entity._Node__node_executable
                        )
                        resolved_name = resolve_substitutions(
                            context, entity._Node__node_name
                        )
                        resolved_namespace = resolve_substitutions(
                            context, entity._Node__node_namespace
                        )
                        resolved_package = resolve_substitutions(
                            context, entity._Node__package
                        )
                        print(
                            f"Found Node: {GREEN}{resolved_executable} {RESET} with the full name: {BLUE}{resolved_namespace or entity._Node__namespace}/{resolved_name or entity._Node__name} {RESET} within package{YELLOW}: {resolved_package}"
                        )
                        nodes.append(entity)
                    elif isinstance(entity, ComposableNode):
                        print(f"Found ComposableNode: {entity}")
                        composable_nodes.append(entity)
                    elif isinstance(entity, ComposableNodeContainer):
                        print(f"Found ComposableNodeContainer: {entity}")
                        containers.append(entity)

                if hasattr(entity, "describe_sub_entities"):
                    sub_entities = entity.describe_sub_entities()
                    if sub_entities:
                        recursively_extract_entities(
                            sub_entities, context, nodes, composable_nodes, containers
                        )
        except LookupError:
            resolved_package = resolve_substitutions(context, entity._Node__package)
            print(f"{RED}package could not be found: {resolved_package}{RESET}")
        except Exception as e:
            print(f"{RED}Error while extracting entities: {e}{RESET}")
            continue


def already_found(entity, nodes, composable_nodes, containers):
    if isinstance(entity, Node):
        if entity in nodes:
            return True
    elif isinstance(entity, ComposableNode):
        if entity in composable_nodes:
            return True
    elif isinstance(entity, ComposableNodeContainer):
        if entity in containers:
            return True
    return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 extract_entities.py <launch_file>")
        sys.exit(1)

    launch_file = sys.argv[1]
    os.chdir(os.path.dirname(launch_file))

    context = LaunchContext()
    source = AnyLaunchDescriptionSource(launch_file)
    ld = source.get_launch_description(context=context)
    ld.visit(context)

    nodes = []
    composable_nodes = []
    containers = []

    recursively_extract_entities(
        ld.entities, context, nodes, composable_nodes, containers
    )

    # If you need to print more details later, you can resolve other attributes the same way:
    # for n in nodes:
    #    resolved_name = resolve_substitutions(context, n._Node__node_name)
    #    print(f"Node name: {resolved_name}")


if __name__ == "__main__":
    main()
