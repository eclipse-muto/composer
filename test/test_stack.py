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
from unittest.mock import patch, MagicMock
from composer.model.stack import Stack


class TestStack(unittest.TestCase):

    def setUp(self):
        self.sample_manifest = {
            "name": "test_stack",
            "context": "test_context",
            "stackId": "test_stack_id",
            "param": [{"name": "param1", "value": "value1"}],
            "arg": [{"name": "arg1", "value": "value1"}],
            "node": [
                {
                    "pkg": "test_pkg",
                    "exec": "test_exec",
                    "name": "test_node",
                    "namespace": "/test_ns",
                    "output": "screen",
                    "param": [{"name": "node_param1", "value": "node_value1"}],
                    "remap": [{"from": "from_topic", "to": "to_topic"}],
                    "args": "--test-arg",
                }
            ],
            "composable": [
                {
                    "package": "test_pkg",
                    "executable": "test_container",
                    "name": "test_container",
                    "namespace": "/test_ns",
                    "node": [
                        {
                            "pkg": "test_pkg",
                            "plugin": "test_plugin",
                            "name": "test_composable_node",
                            "namespace": "/test_ns",
                            "param": [{"name": "comp_param1", "value": "comp_value1"}],
                        }
                    ],
                }
            ],
        }
        self.node = Stack(manifest=self.sample_manifest)

    def tearDown(self):
        # self.node.destroy_node()
        pass

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_initialize(self):
        stack = self.node
        self.assertEqual(stack.name, "test_stack")
        self.assertEqual(stack.context, "test_context")
        self.assertEqual(stack.stackId, "test_stack_id")

        self.assertEqual(len(stack.param), 1)
        self.assertEqual(stack.param[0].name, "param1")
        self.assertEqual(stack.param[0].value, "value1")

        self.assertEqual(len(stack.arg), 1)
        self.assertEqual(stack.arg["arg1"]["name"], "arg1")
        self.assertEqual(stack.arg["arg1"]["value"], "value1")

        self.assertEqual(len(stack.node), 1)
        self.assertEqual(stack.node[0].pkg, "test_pkg")
        self.assertEqual(stack.node[0].exec, "test_exec")

        self.assertEqual(len(stack.composable), 1)
        self.assertEqual(stack.composable[0].package, "test_pkg")
        self.assertEqual(stack.composable[0].executable, "test_container")

    def test_compare_nodes(self):
        stack1 = Stack(manifest=self.sample_manifest)

        modified_manifest = self.sample_manifest.copy()
        modified_manifest["node"].append(
            {
                "pkg": "new_pkg",
                "exec": "new_exec",
                "name": "new_node",
                "namespace": "/new_ns",
            }
        )
        stack2 = Stack(manifest=modified_manifest)

        common, difference, added = stack1.compare_nodes(stack2)

        self.assertEqual(len(common), 1)
        self.assertEqual(list(common)[0].name, "test_node")

        self.assertEqual(len(difference), 0)

        self.assertEqual(len(added), 1)
        self.assertEqual(list(added)[0].name, "new_node")

    def test_compare_composable(self):
        stack1 = Stack(manifest=self.sample_manifest)

        modified_manifest = self.sample_manifest.copy()
        modified_manifest["composable"].append(
            {
                "package": "new_pkg",
                "executable": "new_container",
                "name": "new_container",
                "namespace": "/new_ns",
                "nodes": [
                    {
                        "pkg": "new_pkg",
                        "plugin": "new_plugin",
                        "name": "new_composable_node",
                        "namespace": "/new_ns",
                    }
                ],
            }
        )
        stack2 = Stack(manifest=modified_manifest)

        common, added, removed = stack1.compare_composable(stack2)

        self.assertEqual(len(common), 1)
        self.assertEqual(common[0].name, "test_container")

        self.assertEqual(len(added), 1)
        self.assertEqual(added[0].name, "new_container")

        self.assertEqual(len(removed), 0)

    def test_flatten_nodes(self):
        stack = Stack(manifest=self.sample_manifest)
        flat_nodes = stack.flatten_nodes([])

        self.assertEqual(len(flat_nodes), 1)
        self.assertEqual(flat_nodes[0].name, "test_node")

    def test_flatten_composable(self):
        stack = Stack(manifest=self.sample_manifest)
        flat_composables = stack.flatten_composable([])

        self.assertEqual(len(flat_composables), 1)
        self.assertEqual(flat_composables[0].name, "test_container")
        self.assertEqual(len(flat_composables[0].nodes), 1)
        print(flat_composables[0].nodes)

    def test_compare_ros_params(self):
        params1 = [{"param1": "value1"}, {"param2": "value2"}]
        params2 = [{"param1": "new_value1"}, {"param3": "value3"}]

        differences = Stack.compare_ros_params(params1, params2)

        self.assertEqual(len(differences), 3)
        param_names = {diff["key"] for diff in differences}
        self.assertIn("param1", param_names)
        self.assertIn("param2", param_names)
        self.assertIn("param3", param_names)

    def test_merge(self):
        stack1 = Stack(manifest=self.sample_manifest)

        modified_manifest = self.sample_manifest.copy()
        modified_manifest["node"].append(
            {
                "pkg": "new_pkg",
                "exec": "new_exec",
                "name": "new_node",
                "namespace": "/new_ns",
            }
        )
        stack2 = Stack(manifest=modified_manifest)

        merged_stack = stack1.merge(stack2)

        self.assertEqual(len(merged_stack.node), 2)
        node_names = {n.name for n in merged_stack.node}
        self.assertIn("test_node", node_names)
        self.assertIn("new_node", node_names)

    def test_merge_attributes(self):
        stack1 = Stack(manifest=self.sample_manifest)
        stack2 = Stack(
            manifest={"name": "new_name", "context": "new_context", "stackId": "new_id"}
        )
        merged = Stack(manifest={})

        stack1._merge_attributes(merged, stack2)

        self.assertEqual(merged.name, "new_name")
        self.assertEqual(merged.context, "new_context")
        self.assertEqual(merged.stackId, "new_id")

    def test_merge_nodes(self):
        stack1 = Stack(manifest=self.sample_manifest)
        modified_manifest = self.sample_manifest.copy()
        modified_manifest["node"].append(
            {
                "pkg": "new_pkg",
                "exec": "new_exec",
                "name": "new_node",
                "namespace": "/new_ns",
            }
        )
        stack2 = Stack(manifest=modified_manifest)
        merged = Stack(manifest={})

        stack1._merge_nodes(merged, stack2)

        self.assertEqual(len(merged.node), 2)
        node_actions = {n.name: n.action for n in merged.node}
        self.assertEqual(node_actions["test_node"], "none")
        self.assertEqual(node_actions["new_node"], "start")

    def test_merge_composables(self):
        stack1 = Stack(manifest=self.sample_manifest)
        modified_manifest = self.sample_manifest.copy()
        modified_manifest["composable"].append(
            {
                "package": "new_pkg",
                "executable": "new_container",
                "name": "new_container",
                "namespace": "/new_ns",
            }
        )
        stack2 = Stack(manifest=modified_manifest)
        merged = Stack(manifest={})

        stack1._merge_composables(merged, stack2)

        self.assertEqual(len(merged.composable), 2)
        container_names = {c.name for c in merged.composable}
        self.assertIn("test_container", container_names)
        self.assertIn("new_container", container_names)

    def test_merge_params(self):
        stack1 = Stack(manifest=self.sample_manifest)
        modified_manifest = self.sample_manifest.copy()
        modified_manifest["param"].append({"name": "new_param", "value": "new_value"})
        stack2 = Stack(manifest=modified_manifest)
        merged = Stack(manifest={})

        stack1._merge_params(merged, stack2)

        self.assertEqual(len(merged.param), 2)
        param_names = {p.name for p in merged.param}
        self.assertIn("param1", param_names)
        self.assertIn("new_param", param_names)

    @patch("composer.model.stack.rclpy")
    def test_get_active_nodes(self, mock_rclpy):
        mock_node = MagicMock()
        mock_node.get_node_names_and_namespaces.return_value = [
            ("/ns", "node1"),
            ("/", "node2"),
        ]
        mock_rclpy.create_node.return_value = mock_node

        stack = Stack(manifest=self.sample_manifest)
        active_nodes = stack.get_active_nodes()

        self.assertEqual(len(active_nodes), 2)
        self.assertIn(("/ns", "node1"), active_nodes)
        self.assertIn(("/", "node2"), active_nodes)
        mock_rclpy.create_node.assert_called_once_with(
            "get_active_nodes", enable_rosout=False
        )
        mock_node.destroy_node.assert_called_once()

    @patch("composer.model.stack.Introspector")
    def test_kill_all(self, mock_introspector):
        mock_launcher = MagicMock()
        mock_launcher._active_nodes = [{"node1": 1234}, {"node2": 5678}]

        stack = Stack(manifest=self.sample_manifest)
        stack.kill_all(mock_launcher)

        mock_introspector.return_value.kill.assert_any_call("node1", 1234)
        mock_introspector.return_value.kill.assert_any_call("node2", 5678)

    @patch("composer.model.stack.Introspector")
    def test_kill_diff(self, mock_introspector):
        mock_launcher = MagicMock()
        mock_launcher._active_nodes = [{"test_exec": 1234}, {"other_exec": 5678}]

        stack = Stack(manifest=self.sample_manifest)
        test_node = stack.node[0]
        test_node.action = "stop"

        stack.kill_diff(mock_launcher, stack)

        mock_introspector.return_value.kill.assert_called_once_with("test_exec", 1234)

    @patch("composer.model.stack.subprocess.run")
    def test_change_params_at_runtime(self, mock_run):
        param_differences = {
            ("node1", "node2"): [
                {"key": "param1", "in_node1": "value1", "in_node2": "value2"}
            ]
        }

        stack = Stack(manifest=self.sample_manifest)
        stack.change_params_at_runtime(param_differences)

        mock_run.assert_called_once_with(
            ["ros2", "param", "set", "node1", "param1", "value1"]
        )

    def test_toShallowManifest(self):
        stack = Stack(manifest=self.sample_manifest)
        manifest = stack.toShallowManifest()

        self.assertEqual(manifest["name"], "test_stack")
        self.assertEqual(manifest["context"], "test_context")
        self.assertEqual(manifest["stackId"], "test_stack_id")
        self.assertEqual(manifest["param"], [])
        self.assertEqual(manifest["arg"], [])
        self.assertEqual(manifest["stack"], [])
        self.assertEqual(manifest["composable"], [])
        self.assertEqual(manifest["node"], [])

    def test_toManifest(self):
        stack = Stack(manifest=self.sample_manifest)
        manifest = stack.toManifest()

        self.assertEqual(manifest["name"], "test_stack")
        self.assertEqual(manifest["context"], "test_context")
        self.assertEqual(manifest["stackId"], "test_stack_id")

        self.assertEqual(len(manifest["param"]), 1)
        self.assertEqual(manifest["param"][0]["name"], "param1")
        self.assertEqual(manifest["param"][0]["value"], "value1")

        self.assertEqual(len(manifest["node"]), 1)
        self.assertEqual(manifest["node"][0]["name"], "test_node")

        self.assertEqual(len(manifest["composable"]), 1)
        self.assertEqual(manifest["composable"][0]["name"], "test_container")

    def test_process_remaps(self):
        stack = Stack(manifest=self.sample_manifest)
        remaps_config = [{"from": "from_topic", "to": "to_topic"}]

        processed_remaps = stack.process_remaps(remaps_config)

        self.assertEqual(len(processed_remaps), 1)
        self.assertEqual(processed_remaps[0], ("from_topic", "to_topic"))

    def test_should_node_run(self):
        pass

    def test_load_common_composables(self):
        pass

    def test_handle_composable_nodes(self):
        pass

    def test_handle_regular_nodes(self):
        pass

    def test_handle_managed_nodes(self):
        pass

    @patch("composer.model.stack.Introspector")
    @patch("composer.model.stack.LaunchDescription")
    def test_launch(self, mock_launch_desc, mock_introspector):
        stack = Stack(manifest=self.sample_manifest)
        mock_launcher = MagicMock()

        with patch("composer.model.stack.ComposableNodeContainer"), patch(
            "composer.model.stack.ComposableNode"
        ), patch("composer.model.stack.Node"):
            stack.launch(mock_launcher)

        mock_launcher.start.assert_called_once()

    @patch("composer.model.stack.Introspector")
    @patch("composer.model.stack.LaunchDescription")
    def test_apply(self, mock_launch_desc, mock_introspector):
        stack = Stack(manifest=self.sample_manifest)
        mock_launcher = MagicMock()

        with patch.object(Stack, "kill_diff") as mock_kill_diff, patch.object(
            Stack, "launch"
        ) as mock_launch:
            stack.apply(mock_launcher)

        mock_kill_diff.assert_called_once_with(mock_launcher, stack)
        mock_launch.assert_called_once_with(mock_launcher)

    def test_resolve_expression(self):
        stack = Stack(manifest=self.sample_manifest)

        with patch(
            "composer.model.stack.get_package_share_directory",
            return_value="/fake/path",
        ):
            result = stack.resolve_expression("$(find test_pkg)")
            self.assertEqual(result, "/fake/path")

        with patch.dict("os.environ", {"TEST_VAR": "test_value"}):
            result = stack.resolve_expression("$(env TEST_VAR)")
            self.assertEqual(result, "test_value")

        result = stack.resolve_expression("$(arg arg1)")
        self.assertEqual(result, "value1")

        result = stack.resolve_expression("$(unknown expr)")
        self.assertEqual(result, "$(unknown expr)")

    def test_resolve_param_expression(self):
        pass

    def test_resolve_args(self):
        stack = Stack(manifest={})
        args = [{"name": "test_arg", "value": "test_value"}]

        resolved_args = stack.resolve_args(args)

        self.assertEqual(len(resolved_args), 1)
        self.assertEqual(resolved_args["test_arg"]["name"], "test_arg")
        self.assertEqual(resolved_args["test_arg"]["value"], "test_value")


if __name__ == "__main__":
    unittest.main()
