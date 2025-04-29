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
from unittest.mock import MagicMock, patch
from composer.model.composable import Container


class TestComposable(unittest.TestCase):

    def setUp(self):
        pass

    @patch("composer.model.composable.node")
    def test_toManifest(self, mock_node):
        self.node = Container(MagicMock(), None)
        returned_value = self.node.toManifest()
        expected_value = {
            "package": "",
            "executable": "",
            "name": "",
            "namespace": "",
            "node": [],
            "output": "screen",
            "remap": [],
            "action": "",
        }
        self.assertEqual(returned_value, expected_value)

    def test_resolve_namespace(self):
        self.node = Container(MagicMock(), None)
        self.node.namespace = "test_demo"
        returned_value = self.node.resolve_namespace()
        self.assertEqual(returned_value, "/test_demo//")

        self.node.namespace = "test/demo/"
        returned_value = self.node.resolve_namespace()
        self.assertEqual(returned_value, "/test/demo//")

        self.node.namespace = "test/demo"
        returned_value = self.node.resolve_namespace()
        self.assertEqual(returned_value, "/test/demo//")

        self.node.namespace = "test/demo/core"
        returned_value = self.node.resolve_namespace()
        self.assertEqual(returned_value, "/test/demo/core//")

    def test_eq(self):
        self.node = Container(MagicMock(), None)
        self.node.package = "test_pkg"
        self.node.name = "test_name"
        self.node.namespace = "test_namespace"
        self.node.executable = "test_executable"

        self.other_node = Container(MagicMock(), None)
        self.other_node.package = "test_pkg"
        self.other_node.name = "test_name"
        self.other_node.namespace = "test_namespace"
        self.other_node.executable = "test_executable"

        returned_value = self.node.__eq__(self.other_node)
        self.assertTrue(returned_value)

        self.other_node_diff = Container(MagicMock(), None)
        self.other_node_diff.package = "diff_test_pkg"
        self.other_node_diff.name = "diff_test_name"
        self.other_node_diff.namespace = "diff_test_namespace"
        self.other_node_diff.executable = "diff_test_executable"

        returned_value_diff = self.node.__eq__(self.other_node_diff)
        self.assertFalse(returned_value_diff)


if __name__ == "__main__":
    unittest.main()
