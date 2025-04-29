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

import unittest

from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from composer.introspection.traverser import (
    recursively_extract_entities,
    resolve_substitutions,
)
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution


class TestResolveSubstitutions(unittest.TestCase):
    def test_resolve_text_substitution_list(self):
        context = LaunchContext()
        subs = [TextSubstitution(text="test"), TextSubstitution(text="_node")]
        result = resolve_substitutions(context, subs)
        self.assertEqual(result, "test_node")

    def test_resolve_launch_configuration(self):
        context = LaunchContext()
        DeclareLaunchArgument("test_arg", default_value="test_value").execute(context)
        subs = [LaunchConfiguration("test_arg")]
        result = resolve_substitutions(context, subs)
        self.assertEqual(result, "test_value")

    def test_resolve_string(self):
        context = LaunchContext()
        result = resolve_substitutions(context, "test_node")
        self.assertEqual(result, "test_node")

    def test_resolve_tuple(self):
        context = LaunchContext()
        subs = (TextSubstitution(text="tuple"),)
        result = resolve_substitutions(context, subs)
        self.assertEqual(result, "tuple")


class TestRecursiveExtractEntities(unittest.TestCase):
    def setUp(self):
        self.context = LaunchContext()
        self.nodes = []
        self.composable_nodes = []
        self.containers = []

    def test_simple_node_extraction(self):
        node = Node(package="test_pkg", executable="test_exe")
        recursively_extract_entities(
            [node], self.context, self.nodes, self.composable_nodes, self.containers
        )
        self.assertEqual(len(self.nodes), 1)
        self.assertIsInstance(self.nodes[0], Node)

    def test_composable_node_extraction(self):
        cnode = ComposableNode(
            package="test_pkg", plugin="test_plugin", name="test_node"
        )
        recursively_extract_entities(
            [cnode], self.context, self.nodes, self.composable_nodes, self.containers
        )
        self.assertEqual(len(self.composable_nodes), 1)
        self.assertIsInstance(self.composable_nodes[0], ComposableNode)

    def test_group_action_extraction(self):
        node = Node(package="test_pkg", executable="test_exe")
        group = GroupAction([node])
        recursively_extract_entities(
            [group], self.context, self.nodes, self.composable_nodes, self.containers
        )
        self.assertEqual(len(self.nodes), 1)
        self.assertIsInstance(self.nodes[0], Node)

    def test_duplicate_entity_handling(self):
        node = Node(package="test_pkg", executable="test_exe")
        recursively_extract_entities(
            [node, node],
            self.context,
            self.nodes,
            self.composable_nodes,
            self.containers,
        )
        self.assertEqual(len(self.nodes), 1)


if __name__ == "__main__":
    unittest.main()
