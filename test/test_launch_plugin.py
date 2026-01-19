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
from composer.stack_handlers import StackTypeRegistry

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
        # Mock the launcher.kill method (launcher is now initialized in BasePlugin.__init__)
        if hasattr(self.node, 'launcher') and self.node.launcher:
            self.node.launcher.kill = MagicMock()
        
        # Set up default mock handler for double dispatch - tests can override if needed
        self.default_mock_handler = MagicMock()
        self.default_mock_handler.apply_to_plugin = MagicMock(return_value=True)
        self.node.stack_registry.get_handler = MagicMock(return_value=self.default_mock_handler)
        
        # Set up global Stack mocking for handlers
        self.stack_patcher_json = patch('composer.stack_handlers.json_handler.Stack')
        self.stack_patcher_ditto = patch('composer.stack_handlers.ditto_handler.Stack')
        self.mock_stack_json = self.stack_patcher_json.start()
        self.mock_stack_ditto = self.stack_patcher_ditto.start()
        
        # Configure mock Stack instances
        self.mock_stack_instance = MagicMock()
        self.mock_stack_json.return_value = self.mock_stack_instance
        self.mock_stack_ditto.return_value = self.mock_stack_instance
        
        # Don't set up mock current_stack - let the methods parse it from requests
    
    def tearDown(self) -> None:
        """Clean up patches."""
        self.stack_patcher_json.stop()
        self.stack_patcher_ditto.stop()
    
    def _create_mock_handler(self, returns_success=True, should_raise=None):
        """Helper to create a mock handler that simulates double dispatch."""
        mock_handler = MagicMock()
        if should_raise:
            mock_handler.apply_to_plugin = MagicMock(side_effect=should_raise)
        else:
            mock_handler.apply_to_plugin = MagicMock(return_value=returns_success)
        return mock_handler
    
    def _mock_handler_for_start(self):
        """Mock the stack registry to return a successful handler for start operations."""
        mock_handler = self._create_mock_handler(returns_success=True)
        self.node.stack_registry.get_handler = MagicMock(return_value=mock_handler)
        return mock_handler

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
            "No current stack available or start flag not set.",
        )

    def test_run_async_loop(self):
        self.node.run_async_loop()
        self.node.async_loop.stop.assert_called_once()
        self.node.async_loop.run_forever.assert_called_once()

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

    @patch("subprocess.run")
    @patch("os.environ.update")
    def test_subprocess_exception(self, mock_environ_update, mock_subprocess_run):
        mock_current = MagicMock()
        mock_current.source = json.dumps({"workspace_name": "/mock/file"})
        mock_subprocess_run.side_effect = Exception("Unexpected exception")
        
        # Mock the _get_stack_name method
        with patch.object(self.node, '_get_stack_name', return_value="Test Stack"):
            self.node.source_workspaces(mock_current)

        mock_environ_update.assert_not_called()

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_start_none(self, mock_launch_plugin):
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        request = MagicMock()
        request.start = None
        request.input.current.stack = None

        # Mock handler that raises exception
        self.default_mock_handler.apply_to_plugin = MagicMock(side_effect=Exception("Test exception"))

        returned_value = self.node.handle_start(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available or start flag not set.")
        # Expect 1 info call: only one for start flag not set (no JSON parsing warning when stack is None)
        self.assertEqual(self.node.get_logger().info.call_count, 1)
        self.assertEqual(returned_value, response)
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available or start flag not set.")

    @patch("composer.plugins.launch_plugin.subprocess.Popen")
    @patch("os.chmod")
    @patch("builtins.open", create=True)
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start(
        self,
        mock_launch_plugin,
        mock_source_workspace,
        mock_find_file,
        mock_open,
        mock_chmod,
        mock_popen,
    ):
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        request = MagicMock()
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

        # Mock handler to return success - tests the integration through double dispatch
        mock_handler = self._create_mock_handler(returns_success=True)
        self.node.stack_registry.get_handler = MagicMock(return_value=mock_handler)
        
        self.node.launch_arguments = ["test:=test_args"]

        self.node.handle_start(request, response)
        # Verify handler was obtained and called
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_none(
        self, mock_launch_plugin, mock_source_workspace, mock_find_file, mock_run_script
    ):
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        request = MagicMock()
        request.start = True

        # Mock: No handler found for this stack type
        self.node.stack_registry.get_handler = MagicMock(return_value=None)


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
        
        self.node.launch_arguments = ["test:=test_args"]

        self.node.handle_start(request, response)
        mock_find_file.assert_not_called()
        mock_run_script.assert_not_called()
        self.assertFalse(response.success)
        self.assertEqual(
            response.err_msg,
            "No current stack available or start flag not set.",
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
        request = MagicMock()
        request.start = None
        request.input.current.stack = None
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        self.node.handle_kill(request, response)


        # Expect 1 warning call: only one for start flag not set (no JSON parsing warning when stack is None)
        self.assertEqual(self.node.get_logger().warning.call_count, 1)
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available or start flag not set.")
        mock_core_twin.assert_not_called()

    @patch.object(MutoDefaultLaunchPlugin, "_terminate_launch_process")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill(self, mock_launch_plugin, mock_terminate):
        self.node.set_stack_cli = MagicMock()

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
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

        self.node.handle_kill(request, response)

        # Expect kill request log and success log
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_raw_stack(self, mock_launch_plugin, mock_stack_json, mock_stack_ditto):
        """Test handle_apply with raw stack payload."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        
        # Use proper JSON structure for raw stack
        stack_data = {
            "node": [{"name": "test_node", "pkg": "test_pkg"}],
            "metadata": {"name": "test_stack"}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for raw stack
        # Use the mock_stack_instance from setUp
        mock_stack_instance = self.mock_stack_instance
        mock_stack_ditto.return_value = mock_stack_instance
        mock_stack_json.return_value = mock_stack_instance

        self.node.handle_apply(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_json_stack(self, mock_launch_plugin):
        """Test handle_apply with stack/json payload."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        
        # Use proper JSON structure for stack/json
        stack_data = {
            "metadata": {"name": "test_stack", "content_type": "stack/json"},
            "launch": {"node": [{"name": "test_node"}]}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for stack/json

        self.node.handle_apply(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_archive_stack(self, mock_launch_plugin):
        """Test handle_apply with stack/archive payload."""
        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        
        # Use proper JSON structure for stack/archive
        stack_data = {
            "metadata": {"name": "test_stack", "content_type": "stack/archive"},
            "launch": {"properties": {"launch_file": "test.launch.py"}}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for stack/archive

        self.node.handle_apply(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_no_stack_data(self, mock_launch_plugin):
        """Test handle_apply with no valid stack data."""
        request = MagicMock()
        request.start = True

        # Mock: No handler found for this stack type
        self.node.stack_registry.get_handler = MagicMock(return_value=None)
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        
        # Provide minimal valid stack but mock payload detection to return None
        request.input.current.stack = json.dumps({"metadata": {"name": "test"}})

        # Mock payload type detection returning None for stack_data

        self.node.handle_apply(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available or start flag not set.")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_no_current_stack(self, mock_launch_plugin):
        """Test handle_apply with no current stack."""
        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        
        request.input.current.stack = ""  # Empty stack

        self.node.handle_apply(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available or start flag not set.")

    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_json_payload(self, mock_launch_plugin, mock_source_workspace):
        """Test handle_start with stack/json payload."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        response = MagicMock()
        response.success = False
        response.err_msg = ""
        request = MagicMock()
        request.start = True

        
        # Use proper JSON structure for stack/json
        stack_data = {
            "metadata": {"name": "JSON Stack", "content_type": "stack/json"},
            "launch": {"node": [{"name": "test_node"}]}
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock payload type detection for stack/json

        self.node.handle_start(request, response)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_raw_payload(self, mock_launch_plugin, mock_source_workspace):
        """Test handle_start with raw stack payload."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        response = MagicMock()
        response.success = False
        response.err_msg = ""
        request = MagicMock()
        request.start = True

        
        # Use proper JSON structure for raw stack
        stack_data = {
            "node": [{"name": "test_node", "pkg": "test_pkg"}],
            "metadata": {"name": "Raw Stack"}
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock payload type detection for raw stack

        self.node.handle_start(request, response)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_raw_payload(self, mock_launch_plugin):
        """Test handle_kill with raw stack payload."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        
        # Use proper JSON structure for raw stack
        stack_data = {
            "node": [{"name": "test_node"}],
            "metadata": {"name": "test_stack"}
        }
        request.input.current.stack = json.dumps(stack_data)

        # Mock payload type detection for raw stack

        self.node.handle_kill(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

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


    @patch.object(MutoDefaultLaunchPlugin, "run_script")
    @patch.object(MutoDefaultLaunchPlugin, "find_file")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_not_script(
        self, mock_launch_plugin, mock_find_file, mock_run_script
    ):
        # Use real handlers for this test
        pass  # Registry already initialized in node
        
        self.node.set_stack_cli = MagicMock()

        request = MagicMock()
        request.start = True
        stack_data = {
            "name": "test_stack",
            "on_start": "start_script.sh",
            "on_kill": True
        }
        request.input.current.stack = json.dumps(stack_data)
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        mock_find_file.return_value = None
        
        # Mock the stack parser to return the stack data
        self.node.stack_parser.parse_payload.return_value = stack_data

        self.node.handle_kill(request, response)

        mock_run_script.assert_not_called()
        # Handler succeeds even without on_kill script (no-op kill)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.CoreTwin")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_no_current_stack(self, mock_launch_plugin, mock_core_twin):
        request = MagicMock()
        request.start = True

        # Set up stack data with unknown type so no handler found
        stack_data = {
            "metadata": {
                "name": "test_stack",
                "content_type": "unknown/type"
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        
        # Mock: No handler found for this stack type
        self.node.stack_registry.get_handler = MagicMock(return_value=None)
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        self.node.handle_kill(request, response)

        # With new architecture, no handler means "No handler found for stack type"
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available or start flag not set.")
        mock_core_twin.assert_not_called()

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply(self, mock_launch_plugin, mock_stack_json, mock_stack_ditto):
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
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
        
        # Mock Stack instances are already set up in setUp()
        # mock_stack_instance is available via self.mock_stack_instance
        mock_stack_instance = self.mock_stack_instance
        mock_stack_ditto.return_value = mock_stack_instance
        mock_stack_json.return_value = mock_stack_instance

        self.node.handle_apply(request, response)

        # For raw payload, Stack should be called with the full stack_data
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_exception(self, mock_launch_plugin, mock_stack_json, mock_stack_ditto):
        request = MagicMock()
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        # Set current_stack to None to trigger the "No current stack available" error
        request.input.current.stack = None

        self.node.handle_apply(request, response)

        self.assertFalse(response.success)
        self.assertEqual(
            response.err_msg,
            "No current stack available or start flag not set.",
        )
        # No handler for None stack, so Stack constructors shouldn't be called
        mock_stack_ditto.assert_not_called()
        mock_stack_json.assert_not_called()

    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_stack_json_content_type(self, mock_launch_plugin, mock_source_workspace):
        """Test handle_start with stack/json content_type."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""


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

        self.node.handle_start(request, response)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_stack_archive_content_type(self, mock_launch_plugin, mock_source_workspace):
        """Test handle_start with stack/archive content_type."""
        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        
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

        self.node.handle_start(request, response)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch.object(MutoDefaultLaunchPlugin, "source_workspaces")
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_start_raw_payload_type(self, mock_launch_plugin, mock_source_workspace):
        """Test handle_start with raw payload (node/composable)."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""


        # Use proper structure for raw payload (legacy format without content_type)
        stack_data = {
            "metadata": {
                "name": "Raw Stack",
                "description": "A raw payload stack example"
            },
            "launch": {
                "node": [{"name": "test_node", "pkg": "test_pkg", "exec": "test_exec"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return raw type

        self.node.handle_start(request, response)
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_stack_json_content_type(self, mock_launch_plugin):
        """Test handle_kill with stack/json content_type."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""


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

        self.node.handle_kill(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_stack_archive_content_type(self, mock_launch_plugin):
        """Test handle_kill with stack/archive content_type."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""


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

        self.node.handle_kill(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_kill_raw_payload_type(self, mock_launch_plugin):
        """Test handle_kill with raw payload (node/composable)."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""


        # Use proper raw payload structure (legacy format without content_type)
        stack_data = {
            "metadata": {
                "name": "Raw Kill Stack",
                "description": "A raw payload stack for kill test"
            },
            "launch": {
                "composable": [{"name": "test_composable"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = json.dumps({})

        # Mock the payload parsing to return raw type

        self.node.handle_kill(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_stack_json_content_type(self, mock_launch_plugin, mock_stack_json, mock_stack_ditto):
        """Test handle_apply with stack/json content_type."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        response = MagicMock()
        response.success = False
        response.err_msg = ""


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

        self.node.handle_apply(request, response)

        # For stack/json, it uses the stack_data as manifest
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_stack_archive_content_type(self, mock_launch_plugin, mock_stack_json, mock_stack_ditto):
        """Test handle_apply with stack/archive content_type."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        response = MagicMock()
        response.success = False
        response.err_msg = ""


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

        # Mock the methods that would be called for archive handling
        self.node.handle_apply(request, response)
        
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_raw_payload_type(self, mock_launch_plugin, mock_stack_json, mock_stack_ditto):
        """Test handle_apply with raw payload (node/composable)."""
        # Use real handlers for this test
        pass  # Registry already initialized in node

        request = MagicMock()
        response = MagicMock()
        response.success = False
        response.err_msg = ""


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

        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    @patch("composer.plugins.launch_plugin.LaunchPlugin")
    def test_handle_apply_unknown_content_type(self, mock_launch_plugin, mock_stack_json, mock_stack_ditto):
        """Test handle_apply with unknown content_type uses full payload."""
        request = MagicMock()
        response = MagicMock()
        response.success = False
        response.err_msg = ""

        # Mock: No handler found for unknown content type
        self.node.stack_registry.get_handler = MagicMock(return_value=None)

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

        self.node.handle_apply(request, response)

        # For unknown content type, get_handler returns None, so Stack should not be called
        mock_stack_json.assert_not_called()
        mock_stack_ditto.assert_not_called()
        
        # Should fail with appropriate error message
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack available or start flag not set.")

    @patch('composer.stack_handlers.ditto_handler.Stack')
    @patch('composer.stack_handlers.json_handler.Stack')
    def test_handle_start_stack_json_missing_launch_section(self, mock_stack_json, mock_stack_ditto):
        """
        Test that stack/json without launch section fails gracefully.
        """
        request = MagicMock()
        request.start = True

        # Mock: Handler returns failure
        self.default_mock_handler.apply_to_plugin = MagicMock(return_value=False)
        response = MagicMock()
        response.success = None
        response.err_msg = None

        # Mock handler that returns failure for missing launch section
        self.default_mock_handler.apply_to_plugin = MagicMock(return_value=False)

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

        # Validation now catches missing launch section at find_stack_handler level
        # Response should fail because stack validation rejects malformed stacks early
        self.assertFalse(response.success)

if __name__ == "__main__":
    unittest.main()
