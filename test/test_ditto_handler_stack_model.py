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
Unit and integration tests for the Ditto stack execution path.
Tests that DittoStackHandler correctly instantiates Stack model and calls
.launch(), .apply(), and .kill() with representative legacy manifests.
"""

import unittest
from unittest.mock import MagicMock, patch

from composer.stack_handlers.ditto_handler import DittoStackHandler
from composer.plugins.base_plugin import StackContext, StackOperation


class TestDittoStackHandlerCanHandle(unittest.TestCase):
    """Tests for DittoStackHandler.can_handle() method."""

    def setUp(self):
        self.handler = DittoStackHandler(logger=MagicMock())

    def test_can_handle_node_based_manifest(self):
        """Test that handler recognizes node-based legacy manifests."""
        payload = {
            "node": [
                {"name": "talker", "pkg": "demo_nodes_cpp", "exec": "talker"},
                {"name": "listener", "pkg": "demo_nodes_cpp", "exec": "listener"}
            ]
        }
        self.assertTrue(self.handler.can_handle(payload))

    def test_can_handle_composable_based_manifest(self):
        """Test that handler recognizes composable-based legacy manifests."""
        payload = {
            "composable": [
                {
                    "name": "container",
                    "pkg": "rclcpp_components",
                    "exec": "component_container",
                    "nodes": [
                        {"name": "talker", "plugin": "demo_nodes_cpp::Talker"}
                    ]
                }
            ]
        }
        self.assertTrue(self.handler.can_handle(payload))

    def test_can_handle_script_based_manifest(self):
        """Test that handler recognizes script-based legacy manifests."""
        payload = {
            "on_start": "/opt/muto/start.sh",
            "on_kill": "/opt/muto/stop.sh"
        }
        self.assertTrue(self.handler.can_handle(payload))

    def test_can_handle_launch_description_source(self):
        """Test that handler recognizes launch_description_source manifests."""
        payload = {
            "launch_description_source": "/opt/ros/launch/demo.launch.py"
        }
        self.assertTrue(self.handler.can_handle(payload))

    def test_can_handle_launch_nested_structure(self):
        """Test that handler recognizes nested launch structures."""
        payload = {
            "launch": {
                "node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]
            }
        }
        self.assertTrue(self.handler.can_handle(payload))

    def test_cannot_handle_new_format_stack_archive(self):
        """Test that handler rejects properly typed stack/archive payloads."""
        payload = {
            "metadata": {"content_type": "stack/archive"},
            "launch": {"data": "base64encoded"}
        }
        self.assertFalse(self.handler.can_handle(payload))

    def test_cannot_handle_empty_payload(self):
        """Test that handler rejects empty payloads."""
        self.assertFalse(self.handler.can_handle({}))
        self.assertFalse(self.handler.can_handle(None))
        self.assertFalse(self.handler.can_handle([]))


class TestDittoStackHandlerStackModelIntegration(unittest.TestCase):
    """Integration tests for Stack model instantiation and method calls."""

    def setUp(self):
        self.handler = DittoStackHandler(logger=MagicMock())

    def _create_context(self, stack_data, operation):
        """Helper to create a StackContext."""
        return StackContext(
            stack_data=stack_data,
            metadata={},
            operation=operation,
            name="test-stack",
            logger=MagicMock(),
            workspace_path="/tmp/muto/test",
            launcher=MagicMock(),
            hash="testhash"
        )

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_start_ditto_instantiates_stack_and_launches(self, mock_stack_class):
        """Test that _start_ditto creates Stack and calls launch()."""
        mock_stack = MagicMock()
        mock_stack_class.return_value = mock_stack

        payload = {
            "node": [
                {"name": "talker", "pkg": "demo_nodes_cpp", "exec": "talker"}
            ]
        }
        context = self._create_context(payload, StackOperation.START)

        result = self.handler._start_ditto(context)

        self.assertTrue(result)
        mock_stack_class.assert_called_once_with(manifest=payload)
        mock_stack.launch.assert_called_once_with(context.launcher)

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_start_ditto_with_nested_launch_structure(self, mock_stack_class):
        """Test that _start_ditto handles nested launch structure."""
        mock_stack = MagicMock()
        mock_stack_class.return_value = mock_stack

        launch_data = {"node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]}
        payload = {"launch": launch_data}
        context = self._create_context(payload, StackOperation.START)

        result = self.handler._start_ditto(context)

        self.assertTrue(result)
        mock_stack_class.assert_called_once_with(manifest=launch_data)
        mock_stack.launch.assert_called_once()

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_kill_ditto_instantiates_stack_and_kills(self, mock_stack_class):
        """Test that _kill_ditto creates Stack and calls kill()."""
        mock_stack = MagicMock()
        mock_stack_class.return_value = mock_stack

        payload = {
            "node": [
                {"name": "talker", "pkg": "demo_nodes_cpp", "exec": "talker"}
            ]
        }
        context = self._create_context(payload, StackOperation.KILL)

        result = self.handler._kill_ditto(context)

        self.assertTrue(result)
        mock_stack_class.assert_called_once_with(manifest=payload)
        mock_stack.kill.assert_called_once()

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_apply_ditto_instantiates_stack_and_applies(self, mock_stack_class):
        """Test that _apply_ditto creates Stack and calls apply()."""
        mock_stack = MagicMock()
        mock_stack_class.return_value = mock_stack

        payload = {
            "node": [
                {"name": "talker", "pkg": "demo_nodes_cpp", "exec": "talker"}
            ]
        }
        context = self._create_context(payload, StackOperation.APPLY)

        result = self.handler._apply_ditto(context)

        self.assertTrue(result)
        mock_stack_class.assert_called_once_with(manifest=payload)
        mock_stack.apply.assert_called_once_with(context.launcher)

    def test_start_script_based_ditto_delegates_to_plugin(self):
        """Test that script-based stacks are delegated (not using Stack model)."""
        payload = {
            "on_start": "/opt/muto/start.sh",
            "on_kill": "/opt/muto/stop.sh"
        }
        context = self._create_context(payload, StackOperation.START)

        result = self.handler._start_ditto(context)

        # Script-based stacks return True but don't use Stack model
        self.assertTrue(result)

    def test_kill_script_based_ditto_delegates_to_plugin(self):
        """Test that script-based stack kill is delegated."""
        payload = {
            "on_start": "/opt/muto/start.sh",
            "on_kill": "/opt/muto/stop.sh"
        }
        context = self._create_context(payload, StackOperation.KILL)

        result = self.handler._kill_ditto(context)

        self.assertTrue(result)

    def test_apply_script_based_ditto_is_noop(self):
        """Test that apply for script-based stacks is a no-op."""
        payload = {
            "on_start": "/opt/muto/start.sh",
            "on_kill": "/opt/muto/stop.sh"
        }
        context = self._create_context(payload, StackOperation.APPLY)

        result = self.handler._apply_ditto(context)

        # Apply returns True but does nothing
        self.assertTrue(result)


class TestDittoStackHandlerApplyToPlugin(unittest.TestCase):
    """Tests for the apply_to_plugin dispatcher method."""

    def setUp(self):
        self.handler = DittoStackHandler(logger=MagicMock())
        self.mock_plugin = MagicMock()
        self.mock_request = MagicMock()
        self.mock_response = MagicMock()

    def _create_context(self, stack_data, operation):
        """Helper to create a StackContext."""
        return StackContext(
            stack_data=stack_data,
            metadata={},
            operation=operation,
            name="test-stack",
            logger=MagicMock(),
            workspace_path="/tmp/muto/test",
            launcher=MagicMock(),
            hash="testhash"
        )

    def test_provision_operation_returns_true(self):
        """Test that PROVISION operation returns True (no-op for ditto)."""
        context = self._create_context({}, StackOperation.PROVISION)

        result = self.handler.apply_to_plugin(
            self.mock_plugin, context, self.mock_request, self.mock_response
        )

        self.assertTrue(result)

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_start_operation_delegates_to_start_ditto(self, mock_stack_class):
        """Test that START operation calls _start_ditto."""
        mock_stack = MagicMock()
        mock_stack_class.return_value = mock_stack

        payload = {"node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]}
        context = self._create_context(payload, StackOperation.START)

        result = self.handler.apply_to_plugin(
            self.mock_plugin, context, self.mock_request, self.mock_response
        )

        self.assertTrue(result)
        mock_stack.launch.assert_called_once()

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_kill_operation_delegates_to_kill_ditto(self, mock_stack_class):
        """Test that KILL operation calls _kill_ditto."""
        mock_stack = MagicMock()
        mock_stack_class.return_value = mock_stack

        payload = {"node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]}
        context = self._create_context(payload, StackOperation.KILL)

        result = self.handler.apply_to_plugin(
            self.mock_plugin, context, self.mock_request, self.mock_response
        )

        self.assertTrue(result)
        mock_stack.kill.assert_called_once()

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_apply_operation_delegates_to_apply_ditto(self, mock_stack_class):
        """Test that APPLY operation calls _apply_ditto."""
        mock_stack = MagicMock()
        mock_stack_class.return_value = mock_stack

        payload = {"node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]}
        context = self._create_context(payload, StackOperation.APPLY)

        result = self.handler.apply_to_plugin(
            self.mock_plugin, context, self.mock_request, self.mock_response
        )

        self.assertTrue(result)
        mock_stack.apply.assert_called_once()

    def test_compose_operation_returns_false(self):
        """Test that COMPOSE operation is not supported for ditto handler."""
        context = self._create_context({}, StackOperation.COMPOSE)

        result = self.handler.apply_to_plugin(
            self.mock_plugin, context, self.mock_request, self.mock_response
        )

        self.assertFalse(result)


class TestDittoStackHandlerErrorHandling(unittest.TestCase):
    """Tests for error handling in DittoStackHandler."""

    def setUp(self):
        self.handler = DittoStackHandler(logger=MagicMock())

    def _create_context(self, stack_data, operation):
        return StackContext(
            stack_data=stack_data,
            metadata={},
            operation=operation,
            name="test-stack",
            logger=MagicMock(),
            workspace_path="/tmp/muto/test",
            launcher=MagicMock(),
            hash="testhash"
        )

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_start_handles_stack_exception(self, mock_stack_class):
        """Test that exceptions during start are handled gracefully."""
        mock_stack_class.side_effect = Exception("Stack creation failed")

        payload = {"node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]}
        context = self._create_context(payload, StackOperation.START)

        result = self.handler._start_ditto(context)

        self.assertFalse(result)
        self.handler.logger.error.assert_called()

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_kill_handles_stack_exception(self, mock_stack_class):
        """Test that exceptions during kill are handled gracefully."""
        mock_stack_class.side_effect = Exception("Stack creation failed")

        payload = {"node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]}
        context = self._create_context(payload, StackOperation.KILL)

        result = self.handler._kill_ditto(context)

        self.assertFalse(result)
        self.handler.logger.error.assert_called()

    @patch('composer.stack_handlers.ditto_handler.Stack')
    def test_apply_handles_stack_exception(self, mock_stack_class):
        """Test that exceptions during apply are handled gracefully."""
        mock_stack_class.side_effect = Exception("Stack creation failed")

        payload = {"node": [{"name": "test", "pkg": "pkg", "exec": "exec"}]}
        context = self._create_context(payload, StackOperation.APPLY)

        result = self.handler._apply_ditto(context)

        self.assertFalse(result)
        self.handler.logger.error.assert_called()

    def test_start_with_no_recognizable_structure(self):
        """Test that start with unrecognizable structure returns False."""
        payload = {"unknown_field": "value"}
        context = self._create_context(payload, StackOperation.START)

        result = self.handler._start_ditto(context)

        self.assertFalse(result)


if __name__ == "__main__":
    unittest.main()
