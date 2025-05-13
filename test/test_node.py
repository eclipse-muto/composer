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
import rclpy
from unittest.mock import MagicMock, patch

from composer.model.node import Node
from lifecycle_msgs.srv import (
    GetState,
    GetAvailableTransitions,
    GetAvailableStates,
    ChangeState,
)


class TestNode(unittest.TestCase):

    def setUp(self):
        self.stack_mock = MagicMock()
        self.node_test_toManifest = {
            "env": [],
            "param": [],
            "remap": [],
            "pkg": "test_pkg",
            "lifecycle": "test_lifecycle",
            "exec": "test_exec",
            "plugin": "test_plugin",
            "name": "test",
            "ros_args": "",
            "args": "",
            "namespace": "composable",
            "launch-prefix": None,
            "output": "test",
            "if": "",
            "unless": "",
            "action": "test_start",
            "lifecycle": "start_test",
        }
        self.node_test_toManifest["args"] = self.stack_mock.resolve_expression(
            self.node_test_toManifest.get("args", "")
        )

        self.node = Node(
            stack=self.stack_mock, manifest=self.node_test_toManifest, container=None
        )

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_toManifest(self):
        returned_value = self.node.toManifest()
        self.assertEqual(self.node_test_toManifest, returned_value)

    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_change_state(self, mock_create_node, mock_spin):
        self.node.change_state(["configure"])
        mock_create_node.assert_called_once_with("change_state_node")
        mock_create_node().create_client.assert_called_once()
        mock_spin.assert_called_once()
        mock_create_node().destroy_node.assert_called_once()

    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_get_stack(self, mock_create_node, mock_spin):
        returned_value = self.node.get_state()

        mock_create_node.assert_called_once_with("get_state_node")
        mock_create_node().create_client.assert_called_once_with(
            GetState, "/composable/test/get_state"
        )
        mock_create_node().create_client().call_async.assert_called_once_with(
            GetState.Request()
        )
        mock_spin.assert_called_once_with(
            mock_create_node(),
            mock_create_node().create_client().call_async(),
            timeout_sec=3,
        )
        mock_create_node().destroy_node.assert_called_once()
        self.assertEqual(
            mock_create_node().create_client().call_async().result(), returned_value
        )

    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_get_available_states(self, mock_create_node, mock_spin):
        returned_value = self.node.get_available_states()
        mock_create_node.assert_called_once_with("get_available_states_node")
        mock_create_node().destroy_node.assert_called_once()
        mock_create_node().create_client.assert_called_once_with(
            GetAvailableStates, "/composable/test/get_available_states"
        )
        mock_create_node().create_client().call_async.assert_called_once_with(
            GetAvailableStates.Request()
        )
        mock_spin.assert_called_once_with(
            mock_create_node(),
            mock_create_node().create_client().call_async(),
            timeout_sec=3,
        )
        self.assertEqual(
            mock_create_node().create_client().call_async().result().available_states,
            returned_value,
        )


if __name__ == "__main__":
    unittest.main()
