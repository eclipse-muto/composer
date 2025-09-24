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
import unittest
import json
from unittest.mock import MagicMock, patch
import rclpy
from std_msgs.msg import String
from composer.plugins.compose_plugin import MutoDefaultComposePlugin


class TestComposePlugin(unittest.TestCase):
    def setUp(self):
        self.node = MutoDefaultComposePlugin()
        self.node.incoming_stack = None
        self.node.composed_stack_publisher = MagicMock()
        self.node.get_logger = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_handle_raw_stack(self):
        self.node.incoming_stack = None
        distro = os.environ.get('ROS_DISTRO', 'humble')
        stack_msg = String(
            data=f'{{"name": "Muto Run Rototui from repo", "context": "eclipse_muto", "stackId": "org.eclipse.muto.sandbox:muto_repo_test_stack",  "url": "https://test_url",  "source": {{"ros": "/opt/ros/{distro}/setup.bash", "workspace": "/workspaces/install/setup.bash"}}, "args": {{"launch_muto": "false", "launch_fms": "false", "launch_record_rosbag": "false", "launch_safety_plc": "false", "launch_logger": "false", "launch_css": "false", "vehicle_model": "rototui_vehicle", "sensor_model": "rototui_sensor_kit", "lanelet2_map_file": "64_prod_lanelet_finetuning.osm", "pointcloud_map_file": "pointcloud_map.pcd", "rviz_respawn": "false"}} }}'
        )
        self.node.handle_raw_stack(stack_msg)
        self.assertEqual(self.node.incoming_stack, json.loads(stack_msg.data))

    @patch.object(MutoDefaultComposePlugin, "parse_stack")
    def test_publish_composed_stack(self, mock_parse_stack):
        self.node.incoming_stack = "MockStack"

        self.node.publish_composed_stack()
        mock_parse_stack.assert_called_once_with(self.node.incoming_stack)
        self.node.composed_stack_publisher.publish.assert_called_once_with(
            mock_parse_stack()
        )

    @patch("composer.plugins.compose_plugin.ComposePlugin")
    @patch.object(MutoDefaultComposePlugin, "publish_composed_stack")
    def test_handle_compose(self, mock_publish_composed_stack, mock_compose_plugin):
        request = mock_compose_plugin.request
        request.start = True
        response = mock_compose_plugin.response
        response.success = None
        response.err_msg = None
        self.node.incoming_stack = "MockStack"
        self.node.handle_compose(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")
        mock_publish_composed_stack.assert_called_once()

    @patch("composer.plugins.compose_plugin.ComposePlugin")
    @patch.object(MutoDefaultComposePlugin, "publish_composed_stack")
    def test_handle_compose_start_not_set(
        self, mock_publish_composed_stack, mock_compose_plugin
    ):
        request = mock_compose_plugin.request
        request.start = None
        response = mock_compose_plugin.response(success=None, err_msg=None)
        self.node.incoming_stack = "MockStack"
        self.node.handle_compose(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Start flag not set in request.")
        mock_publish_composed_stack.assert_not_called()
        self.node.get_logger().warn.assert_called_once_with(
            "Start flag not set in compose request."
        )

    @patch.object(
        MutoDefaultComposePlugin,
        "publish_composed_stack",
        side_effect=Exception("dummy_exception"),
    )
    @patch("composer.plugins.compose_plugin.ComposePlugin")
    def test_handle_compose_exception(self, mock_compose_plugin, mock_publish):
        request = mock_compose_plugin.Request()
        request.start = True
        response = mock_compose_plugin.Response()
        self.node.incoming_stack = "MockStack"

        response = self.node.handle_compose(request, response)

        self.assertFalse(response.success)
        self.assertIn("dummy_exception", response.err_msg)
        self.node.get_logger().error.assert_called_once_with(
            "Exception during compose: dummy_exception"
        )
        mock_publish.assert_called_once_with()

    @patch("composer.plugins.compose_plugin.StackManifest")
    def test_parse_stack(self, mock_stack_manifest):
        distro = os.environ.get('ROS_DISTRO', 'humble')
        stack_msg = String(
            data=f'{{"name": "Muto Run Rototui from repo", "context": "eclipse_muto", "stackId": "org.eclipse.muto.sandbox:muto_repo_test_stack",  "url": "https://test_url",  "source": {{"ros": "/opt/ros/{distro}/setup.bash", "workspace": "/workspaces/install/setup.bash"}}, "args": {{"launch_muto": "false", "launch_fms": "false", "launch_record_rosbag": "false", "launch_safety_plc": "false", "launch_logger": "false", "launch_css": "false", "vehicle_model": "rototui_vehicle", "sensor_model": "rototui_sensor_kit", "lanelet2_map_file": "64_prod_lanelet_finetuning.osm", "pointcloud_map_file": "pointcloud_map.pcd", "rviz_respawn": "false"}} }}'
        )
        self.node.incoming_stack = json.loads(stack_msg.data)

        self.node.parse_stack(self.node.incoming_stack)

        mock_stack_manifest.assert_called_once()
        self.assertEqual(
            mock_stack_manifest().name, self.node.incoming_stack.get("name", "")
        )
        self.assertEqual(
            mock_stack_manifest().context, self.node.incoming_stack.get("context", "")
        )
        self.assertEqual(
            mock_stack_manifest().stack_id, self.node.incoming_stack.get("stackId", "")
        )
        self.assertEqual(
            mock_stack_manifest().url, self.node.incoming_stack.get("url", "")
        )
        self.assertEqual(
            mock_stack_manifest().branch, self.node.incoming_stack.get("branch", "")
        )
        self.assertEqual(
            mock_stack_manifest().launch_description_source,
            self.node.incoming_stack.get("launch_description_source", ""),
        )
        self.assertEqual(
            mock_stack_manifest().on_start, self.node.incoming_stack.get("on_start", "")
        )
        self.assertEqual(
            mock_stack_manifest().on_kill, self.node.incoming_stack.get("on_kill", "")
        )
        self.assertEqual(
            mock_stack_manifest().args,
            json.dumps(self.node.incoming_stack.get("args", "")),
        )
        self.assertEqual(
            mock_stack_manifest().source,
            json.dumps(self.node.incoming_stack.get("source", "")),
        )

    if __name__ == "__main__":
        unittest.main()
