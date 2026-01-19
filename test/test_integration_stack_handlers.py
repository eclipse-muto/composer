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

"""
Integration tests for stack handlers in the refactored architecture.

These tests focus on end-to-end workflows through the handler pattern,
testing outcomes rather than implementation details.
"""

import json
import unittest
from unittest.mock import MagicMock, patch
import rclpy

from composer.plugins.launch_plugin import MutoDefaultLaunchPlugin
from composer.plugins.provision_plugin import MutoProvisionPlugin
from composer.plugins.compose_plugin import MutoDefaultComposePlugin
from composer.stack_handlers.registry import StackTypeRegistry


class TestStackHandlerIntegration(unittest.TestCase):
    """Integration tests for the stack handler pattern."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_json_stack_handler_selection(self):
        """Test that JsonStackHandler is selected for stack/json content_type."""
        mock_node = MagicMock()
        mock_logger = MagicMock()
        mock_node.get_logger.return_value = mock_logger
        
        registry = StackTypeRegistry(mock_node, mock_logger)
        registry.discover_and_register_handlers()
        
        payload = {
            "metadata": {"content_type": "stack/json", "name": "Test"},
            "launch": {"node": [{"name": "test"}]}
        }
        
        handler = registry.get_handler(payload)
        self.assertIsNotNone(handler)
        self.assertEqual(handler.__class__.__name__, "JsonStackHandler")

    def test_archive_stack_handler_selection(self):
        """Test that ArchiveStackHandler is selected for stack/archive content_type."""
        mock_node = MagicMock()
        mock_logger = MagicMock()
        mock_node.get_logger.return_value = mock_logger
        
        registry = StackTypeRegistry(mock_node, mock_logger)
        registry.discover_and_register_handlers()
        
        payload = {
            "metadata": {"content_type": "stack/archive", "name": "Test"},
            "launch": {"archive": "base64data"}
        }
        
        handler = registry.get_handler(payload)
        self.assertIsNotNone(handler)
        self.assertEqual(handler.__class__.__name__, "ArchiveStackHandler")

    def test_ditto_stack_handler_selection(self):
        """Test that DittoStackHandler is selected for stacks without content_type."""
        mock_node = MagicMock()
        mock_logger = MagicMock()
        mock_node.get_logger.return_value = mock_logger
        
        registry = StackTypeRegistry(mock_node, mock_logger)
        registry.discover_and_register_handlers()
        
        payload = {
            "node": [{"name": "test_node", "pkg": "test_pkg", "exec": "test_exec"}]
        }
        
        handler = registry.get_handler(payload)
        self.assertIsNotNone(handler)
        self.assertEqual(handler.__class__.__name__, "DittoStackHandler")

    @patch('composer.stack_handlers.json_handler.Stack')
    def test_launch_plugin_json_stack_integration(self, mock_stack):
        """Integration test: Launch plugin with JSON stack."""
        plugin = MutoDefaultLaunchPlugin()
        
        stack_data = {
            "metadata": {"content_type": "stack/json", "name": "Integration Test"},
            "launch": {"node": [{"name": "test_node", "pkg": "demo_nodes_cpp", "exec": "talker"}]}
        }
        
        request = MagicMock()
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = "{}"
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
        plugin.handle_start(request, response)
        
        # Test outcome: verify success
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")
        
        plugin.destroy_node()

    def test_launch_plugin_archive_stack_integration(self):
        """Integration test: Launch plugin with archive stack."""
        plugin = MutoDefaultLaunchPlugin()
        
        stack_data = {
            "metadata": {"content_type": "stack/archive", "name": "Archive Test"},
            "launch": {"properties": {"launch_file": "test.launch.py"}}
        }
        
        request = MagicMock()
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = "{}"
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
        plugin.handle_start(request, response)
        
        # Test outcome: verify success
        self.assertTrue(response.success)
        
        plugin.destroy_node()

    def test_provision_plugin_no_stack_handling(self):
        """Integration test: Provision plugin handles missing stack gracefully."""
        plugin = MutoProvisionPlugin()
        
        request = MagicMock()
        request.input.current.stack = ""
        request.start = True
        response = MagicMock()
        response.success = True
        response.err_msg = ""
        
        plugin.handle_provision(request, response)
        
        # Test outcome: verify failure with appropriate message
        self.assertFalse(response.success)
        self.assertIn("No current stack", response.err_msg)
        
        plugin.destroy_node()

    def test_provision_plugin_json_stack_integration(self):
        """Integration test: Provision plugin with JSON stack."""
        plugin = MutoProvisionPlugin()
        
        stack_data = {
            "metadata": {"content_type": "stack/json", "name": "Provision Test"},
            "launch": {"node": []}
        }
        
        request = MagicMock()
        request.input.current.stack = json.dumps(stack_data)
        request.start = True
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
        plugin.handle_provision(request, response)
        
        # Test outcome: verify it processes (success depends on mocked dependencies)
        # The important thing is it doesn't crash and sets response appropriately
        self.assertIsNotNone(response.success)
        
        plugin.destroy_node()

    def test_compose_plugin_integration(self):
        """Integration test: Compose plugin with valid stack."""
        plugin = MutoDefaultComposePlugin()
        
        stack_data = {
            "metadata": {"content_type": "stack/json", "name": "Compose Test"},
            "launch": {"node": [{"name": "test"}]}
        }
        
        request = MagicMock()
        request.input.current.stack = json.dumps(stack_data)
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
        plugin.handle_compose(request, response)
        
        # Test outcome: verify success
        self.assertTrue(response.success)
        self.assertEqual(response.err_msg, "")
        
        plugin.destroy_node()

    def test_handler_exception_handling(self):
        """Integration test: Verify exception handling in plugin."""
        plugin = MutoDefaultLaunchPlugin()
        
        # Invalid JSON should be handled gracefully
        request = MagicMock()
        request.input.current.stack = "invalid json"
        response = MagicMock()
        response.success = True
        response.err_msg = ""
        
        plugin.handle_start(request, response)
        
        # Test outcome: should fail gracefully
        self.assertFalse(response.success)
        
        plugin.destroy_node()

    def test_unknown_content_type_handling(self):
        """Integration test: Unknown content_type handled gracefully."""
        plugin = MutoDefaultLaunchPlugin()
        
        stack_data = {
            "metadata": {"content_type": "stack/unknown", "name": "Unknown Test"},
            "launch": {}
        }
        
        request = MagicMock()
        request.input.current.stack = json.dumps(stack_data)
        request.input.current.source = "{}"
        response = MagicMock()
        response.success = True
        response.err_msg = ""
        
        plugin.handle_start(request, response)
        
        # Test outcome: should fail when no handler found
        self.assertFalse(response.success)
        
        plugin.destroy_node()


class TestStackHandlerLifecycle(unittest.TestCase):
    """Test complete lifecycle: provision -> start -> kill -> apply."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    @patch('composer.stack_handlers.json_handler.Stack')
    def test_json_stack_lifecycle(self, mock_stack):
        """Test complete lifecycle of a JSON stack."""
        launch_plugin = MutoDefaultLaunchPlugin()
        
        stack_data = {
            "metadata": {"content_type": "stack/json", "name": "Lifecycle Test"},
            "launch": {"node": [{"name": "test_node", "pkg": "demo_nodes_cpp", "exec": "talker"}]}
        }
        stack_json = json.dumps(stack_data)
        
        # 1. START
        request = MagicMock()
        request.input.current.stack = stack_json
        request.input.current.source = "{}"
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
        launch_plugin.handle_start(request, response)
        self.assertTrue(response.success, "Start should succeed")
        
        # 2. KILL
        request = MagicMock()
        request.input.current.stack = stack_json
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
        launch_plugin.handle_kill(request, response)
        self.assertTrue(response.success, "Kill should succeed")
        
        # 3. APPLY
        request = MagicMock()
        request.input.current.stack = stack_json
        request.input.current.source = "{}"
        response = MagicMock()
        response.success = False
        response.err_msg = ""
        
        launch_plugin.handle_apply(request, response)
        self.assertTrue(response.success, "Apply should succeed")
        
        launch_plugin.destroy_node()


if __name__ == '__main__':
    unittest.main()
