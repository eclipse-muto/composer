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

import json
import os
import subprocess
import unittest
from unittest.mock import MagicMock, patch

import rclpy

from composer.plugins.launch_plugin import MutoDefaultLaunchPlugin


class TestLaunchPlugin(unittest.TestCase):

    def setUp(self) -> None:
        self.node = MutoDefaultLaunchPlugin()
        self.node.async_loop = MagicMock()
        self.node.get_logger = MagicMock()
        self.node.current_stack = MagicMock()

    def tearDown(self) -> None:
        self.node.destroy_node()

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    @patch("composer.plugins.launch_plugin.MutoDefaultLaunchPlugin.source_workspaces")
    @patch("os.chdir")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_exception(self, mock_launch_plugin, mock_os, mock_ws):
        mock_launch_plugin.request = None
        mock_launch_plugin.response(success=None, err_msg="")
        self.node.current_stack.native.native_mode = "local"
        self.node.launch_arguments = ["test:=mock"]
        self.node.ws_full_path = MagicMock()
        self.node.launcher_full_path = MagicMock()
        self.node.handle_start(mock_launch_plugin.request, mock_launch_plugin.response)
        mock_os.assert_not_called()
        mock_ws.assert_not_called()
        self.assertFalse(mock_launch_plugin.response.success)
        self.assertEqual(
            mock_launch_plugin.response.err_msg,
            "'NoneType' object has no attribute 'start'",
        )

    def test_run_async_loop(self):
        self.node.run_async_loop()
        self.node.async_loop.stop.assert_called_once()
        self.node.async_loop.run_forever.assert_called_once()

    @patch("composer.plugins.launch_plugin.StackManifest")
    def test_get_stack(self, mock_stack_manifest):
        mock_stack_manifest.args = json.dumps(
            {
                "name": "Muto Run Rototui from repo",
                "context": "eclipse_muto",
                "stackId": "org.eclipse.muto.sandbox:muto_repo_test_stack",
                "url": "https://test_url",
                "source": {
                    "ros": "/opt/ros/humble/setup.bash",
                    "workspace": "/workspaces/install/setup.bash",
                },
                "args": {
                    "launch_muto": "false",
                    "vehicle_model": "rototui_vehicle",
                    "sensor_model": "rototui_sensor_kit",
                    "rviz_respawn": "false",
                },
            }
        )
        self.node.get_stack(mock_stack_manifest)


    @patch("composer.plugins.launch_plugin.StackManifest")
    def test_get_stack_exception(self, mock_stack_manifest):
        mock_stack_manifest = "exception_test_string"
        self.node.get_stack(mock_stack_manifest)

        self.node.get_logger().error.assert_called_once()
        error_args = self.node.get_logger().error.call_args[0][0]
        self.assertRegex(error_args, r"^Unexpected error: .*")

    @patch("composer.plugins.launch_plugin.StackManifest")
    def test_get_stack_json_decode_error(self, mock_stack_manifest):
        stack_msg = MagicMock()
        stack_msg.args = "this is not valid json"
        self.node.get_stack(stack_msg)

        self.node.get_logger().error.assert_called_once()
        error_call_args = self.node.get_logger().error.call_args[0][0]
        self.assertIn("Error parsing launch arguments:", error_call_args)
        mock_stack_manifest.assert_not_called()

    @patch("os.walk")
    @patch("os.path")
    def test_find_file(self, mock_path, mock_walk):
        mock_walk.return_value = [
            ("/ws", ("muto",), ("src", "test")),
            ("/ws/muto", (), ("composer",)),
        ]
        mock_ws_path = "/ws"
        mock_file_name = "composer"
        returned_value = self.node.find_file(mock_ws_path, mock_file_name)
        self.assertIsNotNone(returned_value)
        mock_walk.assert_called_once_with("/ws")
        self.assertEqual(self.node.get_logger().info.call_count, 2)
        mock_path.assert_not_called()

    @patch("os.walk")
    @patch("os.path")
    def test_find_file_not_found(self, mock_path, mock_walk):
        mock_walk.return_value = [
            ("/ws", ("muto",), ("src", "test")),
            ("/ws/muto", (), ("composer",)),
        ]
        mock_ws_path = "/ws"
        mock_file_name = "muto_composer"
        returned_value = self.node.find_file(mock_ws_path, mock_file_name)
        self.assertIsNone(returned_value)
        mock_walk.assert_called_once_with("/ws")
        self.assertEqual(self.node.get_logger().info.call_count, 1)
        self.node.get_logger().warning.assert_called_once_with(
            "File 'muto_composer' not found under '/ws'."
        )
        mock_path.assert_not_called()

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_source_workspaces_no_current_stack(
        self, mock_environ_update, mock_subprocess_run
    ):
        self.node.current_stack = None
        self.node.source_workspaces()

        self.node.get_logger().error.assert_called_with(
            "No current stack available to source workspaces."
        )
        mock_subprocess_run.assert_not_called()
        mock_environ_update.assert_not_called()

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_source_workspace(self, mock_environ_update, mock_subprocess_run):
        self.node.current_stack.name = "Test Stack"
        self.node.current_stack.source = json.dumps({"workspace_name": "/mock/file"})

        self.node.source_workspaces()

        self.node.get_logger().info.assert_called_with(
            "Sourced workspace: workspace_name"
        )
        mock_environ_update.assert_called_once_with({})
        mock_subprocess_run.assert_called_once_with(
            "bash -c 'source /mock/file && env'",
            stdout=-1,
            shell=True,
            executable="/bin/bash",
            cwd="/var/tmp/muto_workspaces/Test_Stack",
            check=True,
            text=True,
        )

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_subprocess_failure(self, mock_environ_update, mock_subprocess_run):
        self.node.current_stack.name = "Test Stack"
        self.node.current_stack.source = json.dumps({"workspace_name": "/mock/file"})

        mock_subprocess_run.side_effect = subprocess.CalledProcessError(
            returncode=1, cmd=""
        )

        self.node.source_workspaces()
        mock_environ_update.assert_not_called()
        self.node.get_logger().error.assert_called_once_with(
            "Failed to source workspace 'workspace_name': None"
        )

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_subprocess_exception(self, mock_environ_update, mock_subprocess_run):
        self.node.current_stack.name = "Test Stack"
        self.node.current_stack.source = json.dumps({"workspace_name": "/mock/file"})
        mock_subprocess_run.side_effect = Exception("Unexpected exception")
        self.node.source_workspaces()
        self.node.get_logger().error.assert_called_with(
            "Error sourcing workspace 'workspace_name': Unexpected exception"
        )

        mock_environ_update.assert_not_called()

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_start_none(self, mock_launch_plugin):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = None

        returned_value = self.node.handle_start(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Start flag not set in request.")
        self.node.get_logger().warning.assert_called_once_with(
            "Start flag not set in start request."
        )
        self.assertEqual(returned_value, response)
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Start flag not set in request.")

    @patch("asyncio.run_coroutine_threadsafe")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start(
        self,
        mock_launch_plugin,
        mock_source_workspace,
        mock_find_file,
        mock_patch_asyncio,
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True

        self.node.current_stack.launch_description_source = (
            "mock_launch_description_source"
        )
        self.node.launch_arguments = ["test:=test_args"]

        self.node.handle_start(request, response)
        mock_source_workspace.assert_called_once_with()
        mock_find_file.assert_called_once()
        mock_patch_asyncio.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("asyncio.run_coroutine_threadsafe")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_file_not_found(
        self,
        mock_launch_plugin,
        mock_source_workspace,
        mock_find_file,
        mock_patch_asyncio,
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True

        self.node.current_stack.launch_description_source = (
            "mock_launch_description_source"
        )
        self.node.launch_arguments = ["test:=test_args"]

        mock_find_file.return_value = None
        self.node.handle_start(request, response)
        self.assertRaises(FileNotFoundError)
        mock_source_workspace.assert_called_once_with()
        mock_find_file.assert_called()
        mock_patch_asyncio.assert_not_called()
        self.assertFalse(response.success)
        self.assertEqual(
            response.err_msg, "Launch file not found: mock_launch_description_source"
        )

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_current_stack_on_start(
        self, mock_launch_plugin, mock_source_workspace, mock_find_file, mock_run_script
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True

        self.node.current_stack.launch_description_source = None
        self.node.launch_arguments = ["test:=test_args"]

        mock_find_file.return_value = "found/path"
        self.node.handle_start(request, response)
        mock_run_script.assert_called_once_with("found/path")
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")
        mock_source_workspace.assert_called_once_with()

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_none(
        self, mock_launch_plugin, mock_source_workspace, mock_find_file, mock_run_script
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True

        self.node.current_stack.launch_description_source = None
        self.node.current_stack.on_start = None
        self.node.launch_arguments = ["test:=test_args"]

        self.node.handle_start(request, response)
        mock_source_workspace.assert_called_once()
        mock_find_file.assert_not_called()
        mock_run_script.assert_not_called()
        self.assertFalse(response.success)
        self.assertEqual(
            response.err_msg, "the JSON object must be str, bytes or bytearray, not MagicMock"
        )

    @patch("subprocess.run")
    @patch("os.path")
    @patch("os.access")
    @patch("os.chmod")
    def test_run_script(self, mock_chmod, mock_access, mock_path, mock_run):
        script_path = "/mock/script/path"
        self.node.run_script(script_path)
        mock_chmod.assert_not_called()
        mock_access.assert_called_once_with(script_path, os.X_OK)
        mock_path.isfile.assert_called_once_with(script_path)
        mock_run.assert_called_once_with(
            ["/mock/script/path"], check=True, capture_output=True, text=True
        )

    @patch("subprocess.run")
    @patch("os.path")
    @patch("os.access")
    @patch("os.chmod")
    def test_run_script_is_not_file(self, mock_chmod, mock_access, mock_path, mock_run):
        mock_path.isfile.return_value = False
        script_path = "/mock/script/path"
        with self.assertRaises(FileNotFoundError):
            self.node.run_script(script_path)
        mock_chmod.assert_not_called()
        mock_access.assert_not_called()
        mock_path.isfile.assert_called_once_with(script_path)
        mock_run.assert_not_called()

    @patch("subprocess.run")
    @patch("os.path")
    @patch("os.access")
    @patch("os.chmod")
    def test_run_script_not_access(self, mock_chmod, mock_access, mock_path, mock_run):
        mock_access.return_value = False
        script_path = "/mock/script/path"
        self.node.run_script(script_path)
        mock_chmod.assert_called_once_with(script_path, 0o755)

        mock_chmod.assert_called_once_with(script_path, 0o755)
        mock_access.assert_called_once_with(script_path, os.X_OK)
        mock_path.isfile.assert_called_once_with(script_path)
        mock_run.assert_called_once_with(
            ["/mock/script/path"], check=True, capture_output=True, text=True
        )


    @patch("composer.plugins.launch_plugin.CoreTwin")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_start_none(self, mock_launch_plugin, mock_core_twin):
        request = mock_launch_plugin.request
        request.start = None
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        self.node.handle_kill(request, response)

        self.node.get_logger().warning.assert_called_once_with(
            "Start flag not set in kill request."
        )
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Start flag not set in request.")
        mock_core_twin.assert_not_called()

    @patch("composer.plugins.launch_plugin.Ros2LaunchParent.kill")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill(self, mock_launch_plugin, mock_ros_launch_kill):
        self.node.set_stack_cli = MagicMock()
        self.node.current_stack.stack_id = "test_stack_id"
        self.node.current_stack.name = "test_stack"
        self.node.current_stack.launch_description_source = "test_launch_file.launch.py"
        self.node.current_stack.on_kill = ""

        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        self.node.handle_kill(request, response)

        self.node.get_logger().info.assert_called_once_with(
            "Launch process killed successfully."
        )
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "Handle kill success")
        mock_ros_launch_kill.assert_called_once_with()

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_on_kill(
        self, mock_launch_plugin, mock_find_file, mock_run_script
    ):
        self.node.set_stack_cli = MagicMock()
        self.node.current_stack.stack_id = "test_stack_id"
        self.node.current_stack.name = "test_stack"
        self.node.current_stack.on_kill = True
        self.node.current_stack.launch_description_source = None

        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        self.node.handle_kill(request, response)

        mock_find_file.assert_called_once_with(
            "/var/tmp/muto_workspaces/test_stack", True
        )
        mock_run_script.assert_called_once_with(mock_find_file())

        self.node.get_logger().info.assert_called_once_with(
            "Kill script executed successfully."
        )
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "Handle kill success")

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_not_script(
        self, mock_launch_plugin, mock_find_file, mock_run_script
    ):
        self.node.set_stack_cli = MagicMock()
        self.node.current_stack.stack_id = "test_stack_id"
        self.node.current_stack.name = "test_stack"
        self.node.current_stack.on_kill = True
        self.node.current_stack.launch_description_source = None

        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        mock_find_file.return_value = None

        self.node.handle_kill(request, response)

        mock_run_script.assert_not_called()
        self.assertRaises(FileNotFoundError)
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Script not found: True")


    @patch("composer.plugins.launch_plugin.CoreTwin")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_no_current_stack(self, mock_launch_plugin, mock_core_twin):
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        self.node.current_stack = None
        self.node.handle_kill(request, response)

        self.node.get_logger().error.assert_called_once_with(
            "No composed stack available. Aborting kill operation."
        )
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No composed stack available.")
        mock_core_twin.assert_not_called()

    @patch("composer.plugins.launch_plugin.Stack")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply(self, mock_launch_plugin, mock_stack):
        request = mock_launch_plugin.request
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        self.node.current_stack.stack = json.dumps({"stack_name": "mock_stack_name"})

        self.node.handle_apply(request, response)

        mock_stack.assert_called_once_with(manifest={'stack_name': 'mock_stack_name'})
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")
        
    @patch("composer.plugins.launch_plugin.Stack")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_exception(self, mock_launch_plugin, mock_stack):
        request = mock_launch_plugin.request
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        self.node.current_stack = MagicMock()

        self.node.handle_apply(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "the JSON object must be str, bytes or bytearray, not MagicMock")
        mock_stack.assert_not_called()

if __name__ == "__main__":
    unittest.main()
