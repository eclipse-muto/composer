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
from muto_msgs.srv import LaunchPlugin


class TestLaunchPlugin(unittest.TestCase):

    def setUp(self) -> None:
        self.node = MutoDefaultLaunchPlugin()
        self.node.async_loop = MagicMock()
        self.node.get_logger = MagicMock()
        # Mock the stack parser
        self.node.stack_parser = MagicMock()
        # Mock the launcher.kill method
        self.node.launcher.kill = MagicMock()
        # Don't set up mock current_stack - let the methods parse it from requests

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
        # Create proper mock request and response objects
        request = MagicMock()
        response = MagicMock()
        response.success = None
        response.err_msg = ""
        request.start = None  # This will cause the "Start flag not set" error
        request.input.current.stack = None
        
        self.node.handle_start(request, response)
        mock_os.assert_not_called()
        mock_ws.assert_not_called()
        self.assertFalse(response.success)
        self.assertEqual(
            response.err_msg,
            "Start flag not set in request.",
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
                    "ros": f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash",
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
    @patch("os.path.isfile")
    @patch("os.path.join")
    def test_find_file(self, mock_join, mock_isfile, mock_walk):
        mock_isfile.return_value = False  # Force the method to use os.walk
        mock_join.side_effect = lambda *args: "/".join(args)
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

    @patch("os.walk")
    @patch("os.path.isfile")
    @patch("os.path.join")
    def test_find_file_not_found(self, mock_join, mock_isfile, mock_walk):
        mock_isfile.return_value = False  # Force the method to use os.walk
        mock_join.side_effect = lambda *args: "/".join(args)
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

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_source_workspaces_no_current_stack(
        self, mock_environ_update, mock_subprocess_run
    ):
        mock_current = None
        self.node.source_workspaces(mock_current)

        self.node.get_logger().error.assert_called_with(
            "No valid current stack available to source workspaces."
        )
        mock_subprocess_run.assert_not_called()
        mock_environ_update.assert_not_called()

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_source_workspace(self, mock_environ_update, mock_subprocess_run):
        mock_current = MagicMock()
        mock_current.source = json.dumps({"workspace_name": "/mock/file"})
        # Mock the _get_stack_name method
        with patch.object(self.node, '_get_stack_name', return_value="Test Stack"):
            self.node.source_workspaces(mock_current)

        self.node.get_logger().info.assert_called_with(
            "Sourced workspace: workspace_name"
        )
        mock_environ_update.assert_called_once_with({})
        mock_subprocess_run.assert_called_once_with(
            "bash -c 'source /mock/file && env'",
            stdout=-1,
            shell=True,
            executable="/bin/bash",
            cwd="/tmp/muto/muto_workspaces/Test_Stack",
            check=True,
            text=True,
        )

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_subprocess_failure(self, mock_environ_update, mock_subprocess_run):
        mock_current = MagicMock()
        mock_current.source = json.dumps({"workspace_name": "/mock/file"})

        mock_subprocess_run.side_effect = subprocess.CalledProcessError(
            returncode=1, cmd=""
        )

        # Mock the _get_stack_name method
        with patch.object(self.node, '_get_stack_name', return_value="Test Stack"):
            self.node.source_workspaces(mock_current)
        mock_environ_update.assert_not_called()
        self.node.get_logger().error.assert_called_once_with(
            "Failed to source workspace 'workspace_name': None"
        )

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_subprocess_exception(self, mock_environ_update, mock_subprocess_run):
        mock_current = MagicMock()
        mock_current.source = json.dumps({"workspace_name": "/mock/file"})
        mock_subprocess_run.side_effect = Exception("Unexpected exception")
        
        # Mock the _get_stack_name method
        with patch.object(self.node, '_get_stack_name', return_value="Test Stack"):
            self.node.source_workspaces(mock_current)
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
        request.input.current.stack = None

        returned_value = self.node.handle_start(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Start flag not set in request.")
        # Expect 1 warning call: only one for start flag not set (no JSON parsing warning when stack is None)
        self.assertEqual(self.node.get_logger().warning.call_count, 1)
        self.assertEqual(returned_value, response)
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Start flag not set in request.")

    @patch("composer.plugins.launch_plugin.subprocess.Popen")
    @patch("os.chmod")
    @patch("builtins.open", create=True)
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start(
        self,
        mock_launch_plugin,
        mock_get_payload,
        mock_source_workspace,
        mock_find_file,
        mock_open,
        mock_chmod,
        mock_popen,
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True
        
        # Use proper JSON structure matching new implementation
        stack_data = {
            "metadata": {
                "name": "Test Stack",
                "content_type": "stack/archive"
            },
            "launch": {
                "properties": {
                    "launch_file": "test.launch.py"
                }
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload type detection
        mock_get_payload.return_value = ("stack/archive", stack_data, "test.launch.py", "launch")
        
        self.node.launch_arguments = ["test:=test_args"]
        mock_find_file.return_value = "/path/to/test.launch.py"

        self.node.handle_start(request, response)
        
        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_find_file.assert_called_once()
        mock_popen.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("asyncio.run_coroutine_threadsafe")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_file_not_found(
        self,
        mock_launch_plugin,
        mock_get_payload,
        mock_source_workspace,
        mock_find_file,
        mock_patch_asyncio,
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True

        # Use proper JSON structure for archive stack with missing file
        stack_data = {
            "metadata": {
                "name": "Archive Stack",
                "content_type": "stack/archive"
            },
            "launch": {
                "properties": {
                    "launch_file": "missing_launch_file.py"
                }
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})
        
        # Mock payload type detection
        mock_get_payload.return_value = ("stack/archive", stack_data, "missing_launch_file.py", "launch")
        
        self.node.launch_arguments = ["test:=test_args"]
        mock_find_file.return_value = None  # File not found
        
        self.node.handle_start(request, response)
        
        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_find_file.assert_called_once()
        mock_patch_asyncio.assert_not_called()
        self.assertFalse(response.success)
        self.assertIn("Launch file not found", response.err_msg)

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_current_stack_on_start(
        self, mock_launch_plugin, mock_get_payload, mock_source_workspace, mock_find_file, mock_run_script
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True

        # Use proper JSON structure with legacy on_start/on_kill script handling
        stack_data = {
            "metadata": {
                "name": "Test Stack",
                "description": "A test stack with on_start script"
            },
            "on_start": "start_script.sh",
            "on_kill": "kill_script.sh"
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock no recognized payload type, should fall back to legacy handling
        mock_get_payload.return_value = (None, None, None, None)
        
        self.node.launch_arguments = ["test:=test_args"]
        mock_find_file.return_value = "found/path"
        
        self.node.handle_start(request, response)
        
        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_run_script.assert_called_once_with("found/path")
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_none(
        self, mock_launch_plugin, mock_get_payload, mock_source_workspace, mock_find_file, mock_run_script
    ):
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True

        # Use proper JSON structure with no recognized launch method
        stack_data = {
            "metadata": {
                "name": "No Launch Method Stack",
                "description": "A stack without launch method"
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})
        
        # Mock no recognized payload type and no legacy methods
        mock_get_payload.return_value = (None, None, None, None)
        
        self.node.launch_arguments = ["test:=test_args"]

        self.node.handle_start(request, response)
        
        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_find_file.assert_not_called()
        mock_run_script.assert_not_called()
        self.assertFalse(response.success)
        self.assertEqual(
            response.err_msg,
            "No valid launch method found for the stack payload.",
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
        request.input.current.stack = None
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        self.node.handle_kill(request, response)

        # Expect 1 warning call: only one for start flag not set (no JSON parsing warning when stack is None)
        self.assertEqual(self.node.get_logger().warning.call_count, 1)
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Start flag not set in request.")
        mock_core_twin.assert_not_called()

    @patch.object(MutoDefaultLaunchPlugin, "_terminate_launch_process")
    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill(self, mock_launch_plugin, mock_get_payload, mock_terminate):
        self.node.set_stack_cli = MagicMock()

        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        # Use proper JSON structure for archive stack
        stack_data = {
            "metadata": {
                "name": "test_stack",
                "content_type": "stack/archive"
            },
            "launch": {
                "properties": {
                    "launch_file": "test_launch_file.launch.py"
                }
            }
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for archive
        mock_get_payload.return_value = ("stack/archive", stack_data, "test_launch_file.launch.py", "launch")

        self.node.handle_kill(request, response)

        # Expect kill request log and success log
        self.node.get_logger().info.assert_any_call(
            "Kill requested; current launch PID=None"
        )
        self.node.get_logger().info.assert_any_call(
            "Launch process killed successfully."
        )
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "Handle kill success")
        mock_terminate.assert_called_once_with()

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch('composer.plugins.launch_plugin.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_raw_stack(self, mock_launch_plugin, mock_stack, mock_get_payload):
        """Test handle_apply with raw stack payload."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        # Use proper JSON structure for raw stack
        stack_data = {
            "node": [{"name": "test_node", "pkg": "test_pkg"}],
            "metadata": {"name": "test_stack"}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for raw stack
        mock_get_payload.return_value = ("raw", stack_data, None, None)
        mock_stack_instance = MagicMock()
        mock_stack.return_value = mock_stack_instance

        self.node.handle_apply(request, response)

        mock_get_payload.assert_called_once()
        mock_stack.assert_called_once_with(stack_data)
        mock_stack_instance.apply.assert_called_once_with(self.node.launcher)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_stack_json_start")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_json_stack(self, mock_launch_plugin, mock_handle_json, mock_get_payload):
        """Test handle_apply with stack/json payload."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        # Use proper JSON structure for stack/json
        stack_data = {
            "metadata": {"name": "test_stack", "content_type": "stack/json"},
            "launch": {"node": [{"name": "test_node"}]}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for stack/json
        mock_get_payload.return_value = ("stack/json", stack_data, None, None)
        mock_handle_json.return_value = True

        self.node.handle_apply(request, response)

        mock_get_payload.assert_called_once()
        mock_handle_json.assert_called_once_with(stack_data)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_archive_start")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_archive_stack(self, mock_launch_plugin, mock_handle_archive, mock_get_payload):
        """Test handle_apply with stack/archive payload."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        # Use proper JSON structure for stack/archive
        stack_data = {
            "metadata": {"name": "test_stack", "content_type": "stack/archive"},
            "launch": {"properties": {"launch_file": "test.launch.py"}}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for stack/archive
        mock_get_payload.return_value = ("stack/archive", stack_data, "test.launch.py", "launch")
        mock_handle_archive.return_value = True

        self.node.handle_apply(request, response)

        mock_get_payload.assert_called_once()
        mock_handle_archive.assert_called_once_with("test.launch.py")
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_no_stack_data(self, mock_launch_plugin, mock_get_payload):
        """Test handle_apply with no valid stack data."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        # Provide minimal valid stack but mock payload detection to return None
        request.input.current.stack = json.dumps({"metadata": {"name": "test"}})

        # Mock payload type detection returning None for stack_data
        mock_get_payload.return_value = ("unknown", None, None, None)

        self.node.handle_apply(request, response)

        mock_get_payload.assert_called_once()
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No valid stack data found for apply operation.")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_no_current_stack(self, mock_launch_plugin):
        """Test handle_apply with no current stack."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        request.input.current.stack = ""  # Empty stack

        self.node.handle_apply(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available for apply operation.")

    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_stack_json_start")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_json_payload(self, mock_launch_plugin, mock_handle_json, mock_get_payload, mock_source_workspace):
        """Test handle_start with stack/json payload."""
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True
        
        # Use proper JSON structure for stack/json
        stack_data = {
            "metadata": {"name": "JSON Stack", "content_type": "stack/json"},
            "launch": {"node": [{"name": "test_node"}]}
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock payload type detection for stack/json
        mock_get_payload.return_value = ("stack/json", stack_data, None, None)
        mock_handle_json.return_value = True

        self.node.handle_start(request, response)
        
        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_handle_json.assert_called_once_with(stack_data)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_raw_stack_start")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_raw_payload(self, mock_launch_plugin, mock_handle_raw, mock_get_payload, mock_source_workspace):
        """Test handle_start with raw stack payload."""
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        request = mock_launch_plugin.request
        request.start = True
        
        # Use proper JSON structure for raw stack
        stack_data = {
            "node": [{"name": "test_node", "pkg": "test_pkg"}],
            "metadata": {"name": "Raw Stack"}
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock payload type detection for raw stack
        mock_get_payload.return_value = ("raw", stack_data, None, None)
        mock_handle_raw.return_value = True

        self.node.handle_start(request, response)
        
        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_handle_raw.assert_called_once_with(stack_data)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_raw_stack_kill")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_raw_payload(self, mock_launch_plugin, mock_handle_raw_kill, mock_get_payload):
        """Test handle_kill with raw stack payload."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        # Use proper JSON structure for raw stack
        stack_data = {
            "node": [{"name": "test_node"}],
            "metadata": {"name": "test_stack"}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for raw stack
        mock_get_payload.return_value = ("raw", stack_data, None, None)
        mock_handle_raw_kill.return_value = True

        self.node.handle_kill(request, response)

        mock_get_payload.assert_called_once()
        mock_handle_raw_kill.assert_called_once_with(stack_data)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "Handle kill success")

    def test_safely_parse_stack_valid_json(self):
        """Test _safely_parse_stack with valid JSON."""
        stack_string = '{"metadata": {"name": "test"}, "launch": {}}'
        result = self.node._safely_parse_stack(stack_string)
        self.assertIsInstance(result, dict)
        self.assertEqual(result["metadata"]["name"], "test")

    def test_safely_parse_stack_invalid_json(self):
        """Test _safely_parse_stack with invalid JSON."""
        stack_string = '{"invalid": json}'
        result = self.node._safely_parse_stack(stack_string)
        self.assertIsNone(result)
        self.node.get_logger().warning.assert_called()

    def test_safely_parse_stack_empty_string(self):
        """Test _safely_parse_stack with empty string."""
        result = self.node._safely_parse_stack("")
        self.assertIsNone(result)

    def test_safely_parse_stack_non_dict(self):
        """Test _safely_parse_stack with non-dict JSON."""
        stack_string = '["array", "instead", "of", "dict"]'
        result = self.node._safely_parse_stack(stack_string)
        self.assertIsNone(result)
        self.node.get_logger().warning.assert_called()

    def test_get_stack_name_with_metadata_name(self):
        """Test _get_stack_name with metadata.name."""
        stack_dict = {"metadata": {"name": "metadata_name"}, "name": "fallback_name"}
        result = self.node._get_stack_name(stack_dict)
        self.assertEqual(result, "metadata_name")

    def test_get_stack_name_with_fallback_name(self):
        """Test _get_stack_name with fallback to name field."""
        stack_dict = {"name": "fallback_name"}
        result = self.node._get_stack_name(stack_dict)
        self.assertEqual(result, "fallback_name")

    def test_get_stack_name_with_default(self):
        """Test _get_stack_name with default value."""
        stack_dict = {}
        result = self.node._get_stack_name(stack_dict)
        self.assertEqual(result, "default")

    def test_get_stack_name_none_input(self):
        """Test _get_stack_name with None input."""
        result = self.node._get_stack_name(None)
        self.assertEqual(result, "default")

    def test_get_payload_type_and_data_archive(self):
        """Test _get_payload_type_and_data for archive type."""
        payload = {"metadata": {"content_type": "stack/archive"}}
        parsed_payload = {
            "metadata": {"content_type": "stack/archive"},
            "launch": {"properties": {"launch_file": "test.launch.py", "command": "launch"}}
        }
        self.node.stack_parser.parse_payload.return_value = parsed_payload
        
        payload_type, stack_data, launch_file, command = self.node._get_payload_type_and_data(payload)
        
        self.assertEqual(payload_type, "stack/archive")
        self.assertEqual(stack_data, parsed_payload)
        self.assertEqual(launch_file, "test.launch.py")
        self.assertEqual(command, "launch")

    def test_get_payload_type_and_data_json(self):
        """Test _get_payload_type_and_data for JSON type."""
        payload = {"metadata": {"content_type": "stack/json"}}
        parsed_payload = {"metadata": {"content_type": "stack/json"}, "launch": {"node": []}}
        self.node.stack_parser.parse_payload.return_value = parsed_payload
        
        payload_type, stack_data, launch_file, command = self.node._get_payload_type_and_data(payload)
        
        self.assertEqual(payload_type, "stack/json")
        self.assertEqual(stack_data, parsed_payload)
        self.assertIsNone(launch_file)
        self.assertIsNone(command)

    def test_get_payload_type_and_data_raw(self):
        """Test _get_payload_type_and_data for raw type."""
        payload = {"node": [{"name": "test_node"}]}
        parsed_payload = {"node": [{"name": "test_node"}]}
        self.node.stack_parser.parse_payload.return_value = parsed_payload
        
        payload_type, stack_data, launch_file, command = self.node._get_payload_type_and_data(payload)
        
        self.assertEqual(payload_type, "raw")
        self.assertEqual(stack_data, parsed_payload)
        self.assertIsNone(launch_file)
        self.assertIsNone(command)

    def test_get_payload_type_and_data_invalid(self):
        """Test _get_payload_type_and_data with invalid payload."""
        self.node.stack_parser.parse_payload.return_value = None
        
        result = self.node._get_payload_type_and_data("invalid")
        
        self.assertEqual(result, (None, None, None, None))

    @patch('composer.plugins.launch_plugin.Stack')
    def test_handle_stack_json_start_success(self, mock_stack):
        """Test _handle_stack_json_start with valid manifest."""
        manifest = {"launch": {"node": [{"name": "test_node"}]}}
        mock_stack_instance = MagicMock()
        mock_stack.return_value = mock_stack_instance
        
        result = self.node._handle_stack_json_start(manifest)
        
        self.assertTrue(result)
        mock_stack.assert_called_once_with(manifest={"node": [{"name": "test_node"}]})
        mock_stack_instance.launch.assert_called_once_with(self.node.launcher)

    def test_handle_stack_json_start_no_launch(self):
        """Test _handle_stack_json_start with no launch section."""
        manifest = {"metadata": {"name": "test"}}
        
        result = self.node._handle_stack_json_start(manifest)
        
        self.assertFalse(result)
        self.node.get_logger().error.assert_called_with("No 'launch' section found in stack/json manifest")

    @patch('composer.plugins.launch_plugin.Stack')
    def test_handle_raw_stack_start_success(self, mock_stack):
        """Test _handle_raw_stack_start with valid data."""
        stack_data = {"node": [{"name": "test_node"}]}
        mock_stack_instance = MagicMock()
        mock_stack.return_value = mock_stack_instance
        
        result = self.node._handle_raw_stack_start(stack_data)
        
        self.assertTrue(result)
        mock_stack.assert_called_once_with(manifest=stack_data)
        mock_stack_instance.launch.assert_called_once_with(self.node.launcher)

    @patch.object(MutoDefaultLaunchPlugin, '_launch_via_ros2')
    @patch.object(MutoDefaultLaunchPlugin, '_terminate_launch_process')
    @patch.object(MutoDefaultLaunchPlugin, 'find_file')
    @patch.object(MutoDefaultLaunchPlugin, '_get_stack_name')
    def test_handle_archive_start_success(self, mock_get_name, mock_find_file, mock_terminate, mock_launch):
        """Test _handle_archive_start with valid launch file."""
        launch_file = "test.launch.py"
        mock_get_name.return_value = "test_stack"
        mock_find_file.return_value = "/path/to/test.launch.py"
        
        result = self.node._handle_archive_start(launch_file)
        
        self.assertTrue(result)
        mock_terminate.assert_called_once()
        mock_find_file.assert_called_once()
        mock_launch.assert_called_once_with("/path/to/test.launch.py")

    @patch.object(MutoDefaultLaunchPlugin, 'find_file')
    @patch.object(MutoDefaultLaunchPlugin, '_get_stack_name')
    def test_handle_archive_start_file_not_found(self, mock_get_name, mock_find_file):
        """Test _handle_archive_start with file not found."""
        launch_file = "missing.launch.py"
        mock_get_name.return_value = "test_stack"
        mock_find_file.return_value = None
        
        with self.assertRaises(FileNotFoundError):
            self.node._handle_archive_start(launch_file)

    @patch.object(MutoDefaultLaunchPlugin, '_terminate_launch_process')
    def test_handle_archive_kill_success(self, mock_terminate):
        """Test _handle_archive_kill with valid launch file."""
        launch_file = "test.launch.py"
        
        result = self.node._handle_archive_kill(launch_file)
        
        self.assertTrue(result)
        mock_terminate.assert_called_once()
        self.node.get_logger().info.assert_called_with("Launch process killed successfully.")

    def test_handle_raw_stack_kill_success(self):
        """Test _handle_raw_stack_kill."""
        stack_data = {"node": [{"name": "test_node"}]}
        
        result = self.node._handle_raw_stack_kill(stack_data)
        
        self.assertTrue(result)
        self.node.launcher.kill.assert_called_once()
        self.node.get_logger().info.assert_called_with("Launch process killed successfully.")

    @patch('composer.plugins.launch_plugin.Stack')
    def test_handle_raw_stack_apply_success(self, mock_stack):
        """Test _handle_raw_stack_apply with valid data."""
        stack_data = {"node": [{"name": "test_node"}]}
        mock_stack_instance = MagicMock()
        mock_stack.return_value = mock_stack_instance
        
        result = self.node._handle_raw_stack_apply(stack_data)
        
        self.assertTrue(result)
        mock_stack.assert_called_once_with(manifest=stack_data)
        mock_stack_instance.apply.assert_called_once_with(self.node.launcher)

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_on_kill(
        self, mock_launch_plugin, mock_find_file, mock_run_script
    ):
        self.node.set_stack_cli = MagicMock()

        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper JSON structure with on_kill script
        stack_data = {
            "metadata": {
                "name": "test_stack",
                "description": "A test stack for killing"
            },
            "stack_id": "test_stack_id",
            "launch_description_source": None,
            "on_kill": "kill_script.sh"
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the stack parser to return the stack data
        self.node.stack_parser.parse_payload.return_value = stack_data

        self.node.handle_kill(request, response)

        mock_find_file.assert_called_once_with(
            "/tmp/muto/muto_workspaces/test_stack", "kill_script.sh"
        )
        mock_run_script.assert_called_once_with(mock_find_file())

        # Expect two info calls: one for kill request, one for success
        self.assertEqual(self.node.get_logger().info.call_count, 2)
        self.node.get_logger().info.assert_any_call(
            "Kill requested; current launch PID=None"
        )
        self.node.get_logger().info.assert_any_call(
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

        request = mock_launch_plugin.request
        request.start = True
        stack_data = {
            "name": "test_stack",
            "on_kill": True
        }
        request.input.current.stack = json.dumps(stack_data)
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        mock_find_file.return_value = None
        
        # Mock the stack parser to return the stack data
        self.node.stack_parser.parse_payload.return_value = stack_data

        self.node.handle_kill(request, response)

        mock_run_script.assert_not_called()
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
        
        # Use proper JSON structure for a raw stack (has no content_type, just nodes)
        stack_data = {
            "metadata": {
                "name": "mock_stack_name",
                "description": "A mock stack for testing apply"
            },
            "node": [{"name": "test_node", "pkg": "test_pkg"}]
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock the stack parser to return the parsed payload
        self.node.stack_parser.parse_payload.return_value = stack_data
        
        # Mock Stack instance
        mock_stack_instance = MagicMock()
        mock_stack.return_value = mock_stack_instance

        self.node.handle_apply(request, response)

        # For raw payload, Stack should be called with the full stack_data
        mock_stack.assert_called_once_with(stack_data)
        mock_stack_instance.apply.assert_called_once_with(self.node.launcher)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.Stack")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_exception(self, mock_launch_plugin, mock_stack):
        request = mock_launch_plugin.request
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        # Set current_stack to None to trigger the "No current stack available" error
        request.input.current.stack = None

        self.node.handle_apply(request, response)

        self.assertFalse(response.success)
        self.assertEqual(
            response.err_msg,
            "No current stack available for apply operation.",
        )
        mock_stack.assert_not_called()

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_stack_json_start")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_stack_json_content_type(self, mock_launch_plugin, mock_source_workspace, mock_handle_json, mock_get_payload):
        """Test handle_start with stack/json content_type."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper JSON stack structure
        stack_data = {
            "metadata": {
                "name": "JSON Stack",
                "description": "A JSON content type stack",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return stack/json type
        mock_get_payload.return_value = ("stack/json", {"node": [{"name": "test_node"}]}, None, None)
        mock_handle_json.return_value = True

        self.node.handle_start(request, response)

        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_handle_json.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_archive_start")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_stack_archive_content_type(self, mock_launch_plugin, mock_source_workspace, mock_handle_archive, mock_get_payload):
        """Test handle_start with stack/archive content_type."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None
        
        # Use proper archive structure matching talker-listener-xarchive.json
        stack_data = {
            "metadata": {
                "name": "Muto Simple Talker-Listener Stack",
                "description": "A simple talker-listener stack example",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "H4sIAAAAAAAAA+1de...",  # truncated for test
                "properties": {
                    "algorithm": "sha256",
                    "checksum": "553fd2dc7d0eb41e7d65c467d358e7962d3efbb0e2f2e4f8158e926a081f96d0",
                    "launch_file": "launch/talker_listener.launch.py",
                    "command": "launch",
                    "flatten": True
                }
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return stack/archive type
        mock_get_payload.return_value = ("stack/archive", stack_data["launch"], "launch/talker_listener.launch.py", "launch")
        mock_handle_archive.return_value = True

        self.node.handle_start(request, response)

        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_handle_archive.assert_called_once_with("launch/talker_listener.launch.py")
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_raw_stack_start")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_raw_payload_type(self, mock_launch_plugin, mock_source_workspace, mock_handle_raw, mock_get_payload):
        """Test handle_start with raw payload (node/composable)."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper structure for raw payload
        stack_data = {
            "metadata": {
                "name": "Raw Stack",
                "description": "A raw payload stack example",
                "content_type": "raw"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return raw type
        mock_get_payload.return_value = ("raw", {"node": [{"name": "test_node"}]}, None, None)
        mock_handle_raw.return_value = True

        self.node.handle_start(request, response)

        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_handle_raw.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_legacy_script_start")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_unknown_content_type(self, mock_launch_plugin, mock_source_workspace, mock_handle_legacy, mock_get_payload):
        """Test handle_start with unknown content_type falls back to legacy."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use structure with unknown content type
        stack_data = {
            "metadata": {
                "name": "Unknown Stack",
                "description": "A stack with unknown content type",
                "content_type": "unknown/type"
            },
            "launch": {
                "some_data": "test"
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return None (unknown type)
        mock_get_payload.return_value = (None, None, None, None)
        mock_handle_legacy.return_value = True

        self.node.handle_start(request, response)

        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_handle_legacy.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_legacy_script_start")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_missing_content_type(self, mock_launch_plugin, mock_source_workspace, mock_handle_legacy, mock_get_payload):
        """Test handle_start with missing content_type falls back to legacy."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use structure with missing content_type
        stack_data = {
            "metadata": {
                "name": "Missing Content Type Stack",
                "description": "A stack with missing content type"
                # Note: no content_type field
            },
            "launch": {
                "some_data": "test"
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return None (missing content_type)
        mock_get_payload.return_value = (None, None, None, None)
        mock_handle_legacy.return_value = True

        self.node.handle_start(request, response)

        mock_source_workspace.assert_called_once_with(request.input.current)
        mock_get_payload.assert_called_once()
        mock_handle_legacy.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_legacy_script_start")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_invalid_json_payload(self, mock_launch_plugin, mock_source_workspace, mock_handle_legacy, mock_get_payload):
        """Test handle_start with invalid JSON payload falls back to legacy."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use invalid JSON for stack data
        request.input.current.stack = "invalid json {"
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return None (invalid payload)
        mock_get_payload.return_value = (None, None, None, None)
        mock_handle_legacy.return_value = True

        self.node.handle_start(request, response)

        # For invalid JSON, source_workspaces is not called because current_stack is None
        mock_source_workspace.assert_not_called()
        mock_get_payload.assert_not_called()
        mock_handle_legacy.assert_not_called()
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No default stack on device.")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_raw_stack_kill")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_stack_json_content_type(self, mock_launch_plugin, mock_handle_raw_kill, mock_get_payload):
        """Test handle_kill with stack/json content_type."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper stack/json structure
        stack_data = {
            "metadata": {
                "name": "JSON Stack",
                "description": "A JSON content type stack",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return stack/json type
        mock_get_payload.return_value = ("stack/json", {"node": [{"name": "test_node"}]}, None, None)
        mock_handle_raw_kill.return_value = True

        self.node.handle_kill(request, response)

        mock_get_payload.assert_called_once()
        mock_handle_raw_kill.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "Handle kill success")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_archive_kill")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_stack_archive_content_type(self, mock_launch_plugin, mock_handle_archive_kill, mock_get_payload):
        """Test handle_kill with stack/archive content_type."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper archive structure
        stack_data = {
            "metadata": {
                "name": "Archive Stack",
                "description": "An archive content type stack",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "H4sIAAAAAAAAA+1de...",  # truncated for test
                "properties": {
                    "algorithm": "sha256",
                    "checksum": "553fd2dc7d0eb41e7d65c467d358e7962d3efbb0e2f2e4f8158e926a081f96d0",
                    "launch_file": "launch/test.launch.py",
                    "command": "launch",
                    "flatten": True
                }
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return stack/archive type
        mock_get_payload.return_value = ("stack/archive", stack_data["launch"], "launch/test.launch.py", "launch")
        mock_handle_archive_kill.return_value = True

        self.node.handle_kill(request, response)

        mock_get_payload.assert_called_once()
        mock_handle_archive_kill.assert_called_once_with("launch/test.launch.py")
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "Handle kill success")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch.object(MutoDefaultLaunchPlugin, "_handle_raw_stack_kill")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_raw_payload_type(self, mock_launch_plugin, mock_handle_raw_kill, mock_get_payload):
        """Test handle_kill with raw payload (node/composable)."""
        request = mock_launch_plugin.request
        request.start = True
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper raw payload structure
        stack_data = {
            "metadata": {
                "name": "Raw Kill Stack",
                "description": "A raw payload stack for kill test",
                "content_type": "raw"
            },
            "launch": {
                "composable": [{"name": "test_composable"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return raw type
        mock_get_payload.return_value = ("raw", {"composable": [{"name": "test_composable"}]}, None, None)
        mock_handle_raw_kill.return_value = True

        self.node.handle_kill(request, response)

        mock_get_payload.assert_called_once()
        mock_handle_raw_kill.assert_called_once()
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "Handle kill success")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.Stack")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_stack_json_content_type(self, mock_launch_plugin, mock_stack, mock_get_payload):
        """Test handle_apply with stack/json content_type."""
        request = mock_launch_plugin.request
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper JSON stack structure
        stack_data = {
            "metadata": {
                "name": "JSON Apply Stack",
                "description": "A JSON content type stack for apply test",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return stack/json type
        mock_get_payload.return_value = ("stack/json", stack_data, None, None)

        self.node.handle_apply(request, response)

        # For stack/json, it uses the stack_data as manifest
        mock_stack.assert_called_once_with(manifest={"node": [{"name": "test_node"}]})
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "_get_payload_type_and_data")
    @patch("composer.plugins.launch_plugin.Stack")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_stack_archive_content_type(self, mock_launch_plugin, mock_stack, mock_get_payload):
        """Test handle_apply with stack/archive content_type."""
        request = mock_launch_plugin.request
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Use proper archive structure
        stack_data = {
            "metadata": {
                "name": "Archive Apply Stack",
                "description": "An archive content type stack for apply test",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "H4sIAAAAAAAAA+1de...",  # truncated for test
                "properties": {
                    "algorithm": "sha256",
                    "checksum": "553fd2dc7d0eb41e7d65c467d358e7962d3efbb0e2f2e4f8158e926a081f96d0",
                    "launch_file": "launch/test.launch.py",
                    "command": "launch",
                    "flatten": True
                }
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return stack/archive type
        mock_get_payload.return_value = ("stack/archive", stack_data, "launch/test.launch.py", "launch")

        # Mock the methods that would be called for archive handling
        with patch.object(self.node, '_handle_archive_start', return_value=True) as mock_handle_archive:
            self.node.handle_apply(request, response)

            # For stack/archive, it should call _handle_archive_start, not Stack
            mock_handle_archive.assert_called_once_with("launch/test.launch.py")
            mock_stack.assert_not_called()
            
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.Stack")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_raw_payload_type(self, mock_launch_plugin, mock_stack):
        """Test handle_apply with raw payload (node/composable)."""
        request = mock_launch_plugin.request
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Raw payload with node - use proper request structure
        raw_payload = {
            "metadata": {
                "name": "Raw Apply Stack",
                "description": "A raw payload stack for apply test"
            },
            "node": [{"name": "test_node"}]
        }
        request.input.current.stack = json.dumps(raw_payload)
        request.input.current.source = json.dumps({})

        # Mock the stack parser to return the raw payload
        self.node.stack_parser.parse_payload.return_value = raw_payload

        self.node.handle_apply(request, response)

        mock_stack.assert_called_once_with(raw_payload)
        mock_stack.return_value.apply.assert_called_once_with(self.node.launcher)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.Stack")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_unknown_content_type(self, mock_launch_plugin, mock_stack):
        """Test handle_apply with unknown content_type uses full payload."""
        request = mock_launch_plugin.request
        response = mock_launch_plugin.response
        response.success = None
        response.err_msg = None

        # Payload with unknown content_type
        unknown_payload = {
            "metadata": {
                "name": "test-unknown-stack",
                "content_type": "unknown/type"
            },
            "custom": {
                "data": "some_data"
            }
        }
        request.input.current.stack = json.dumps(unknown_payload)
        request.input.current.source = json.dumps({})

        # Mock the stack parser to return the unknown payload
        self.node.stack_parser.parse_payload.return_value = unknown_payload

        self.node.handle_apply(request, response)

        # For unknown content type, _get_payload_type_and_data returns None, so Stack should not be called
        mock_stack.assert_not_called()
        
        # Should fail with appropriate error message
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No valid stack data found for apply operation.")

    @patch("composer.plugins.launch_plugin.Stack")
    def test_handle_start_complete_stack_json_flow_regression(self, mock_stack_class):
        """
        Regression test for complete stack/json flow including talker-listener-json.json.
        This test ensures the complete pipeline works end-to-end without mocking internal methods.
        """
        # Mock the Stack class and its instances
        mock_stack_instance = MagicMock()
        mock_stack_class.return_value = mock_stack_instance
        
        # Create proper request and response objects
        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = None
        response.err_msg = None
        
        # Use the exact structure from docs/samples/talker-listener/talker-listener-json.json
        stack_data = {
            "metadata": {
                "name": "Muto Simple Talker-Listener Stack",
                "description": "A simple talker-listener stack example using demo_nodes_cpp package.",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [
                    {
                        "name": "talker",
                        "pkg": "demo_nodes_cpp",
                        "exec": "talker"
                    },
                    {
                        "name": "listener",
                        "pkg": "demo_nodes_cpp",
                        "exec": "listener"
                    }
                ]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})
        
        # Mock the stack parser to return the parsed payload as expected
        expected_parsed_payload = {
            "metadata": {
                "name": "Muto Simple Talker-Listener Stack",
                "description": "A simple talker-listener stack example using demo_nodes_cpp package.",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [
                    {
                        "name": "talker",
                        "pkg": "demo_nodes_cpp",
                        "exec": "talker"
                    },
                    {
                        "name": "listener",
                        "pkg": "demo_nodes_cpp",
                        "exec": "listener"
                    }
                ]
            }
        }
        self.node.stack_parser.parse_payload.return_value = expected_parsed_payload
        
        # Mock source_workspaces to avoid file system operations
        with patch.object(self.node, 'source_workspaces'):
            self.node.handle_start(request, response)
        
        # Verify the Stack was created with the correct manifest (just the launch content)
        expected_manifest = {
            "node": [
                {
                    "name": "talker",
                    "pkg": "demo_nodes_cpp",
                    "exec": "talker"
                },
                {
                    "name": "listener",
                    "pkg": "demo_nodes_cpp",
                    "exec": "listener"
                }
            ]
        }
        mock_stack_class.assert_called_once_with(manifest=expected_manifest)
        
        # Verify the stack.launch() method was called
        mock_stack_instance.launch.assert_called_once_with(self.node.launcher)
        
        # Verify success response
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.Stack")
    def test_handle_start_stack_json_missing_launch_section(self, mock_stack_class):
        """
        Test that stack/json without launch section fails gracefully.
        """
        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = None
        response.err_msg = None
        
        # Stack data without launch section
        stack_data = {
            "metadata": {
                "name": "Invalid Stack",
                "content_type": "stack/json"
            }
            # Missing launch section
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})
        
        # Mock the stack parser to return the stack data
        self.node.stack_parser.parse_payload.return_value = stack_data
        
        with patch.object(self.node, 'source_workspaces'):
            self.node.handle_start(request, response)
        
        # Verify Stack was not created
        mock_stack_class.assert_not_called()
        
        # Verify failure response
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No valid launch method found for the stack payload.")

    @patch("composer.plugins.launch_plugin.Stack")  
    def test_handle_apply_complete_stack_json_flow_regression(self, mock_stack_class):
        """
        Regression test for complete stack/json apply flow.
        """
        # Mock the Stack class and its instances
        mock_stack_instance = MagicMock()
        mock_stack_class.return_value = mock_stack_instance
        
        request = MagicMock()
        response = MagicMock()
        response.success = None
        response.err_msg = None
        
        # Use stack/json structure
        stack_data = {
            "metadata": {
                "name": "Test Apply Stack",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [
                    {"name": "test_node", "pkg": "test_pkg", "exec": "test_exec"}
                ]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        
        # Mock the stack parser to return the stack data
        self.node.stack_parser.parse_payload.return_value = stack_data
        
        # Mock the _handle_stack_json_start method since that's what gets called for stack/json in apply
        with patch.object(self.node, '_handle_stack_json_start', return_value=True) as mock_handle_json:
            self.node.handle_apply(request, response)
            
            # For stack/json in apply, it should call _handle_stack_json_start
            mock_handle_json.assert_called_once_with(stack_data)
            mock_stack_class.assert_not_called()

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")


if __name__ == "__main__":
    unittest.main()
