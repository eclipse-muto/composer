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
from composer.events import EventBus, EventType, StackRequestEvent
from composer.subsystems.stack_manager import (
    StackManager, StackAnalyzer, StackProcessor, StackStateManager,
    StackType, ExecutionRequirements
)


class TestStackAnalyzer(unittest.TestCase):
    
    def setUp(self):
        self.event_bus = EventBus()
        self.logger = MagicMock()
        self.analyzer = StackAnalyzer(self.event_bus, self.logger)
    
    def test_analyze_archive_stack_type(self):
        """Test detection of archive stack type."""
        stack = {
            "metadata": {
                "content_type": "stack/archive"
            }
        }
        
        stack_type = self.analyzer.analyze_stack_type(stack)
        self.assertEqual(stack_type, StackType.ARCHIVE)
    
    def test_analyze_json_stack_type(self):
        """Test detection of JSON stack type."""
        stack = {
            "metadata": {
                "content_type": "stack/json"
            }
        }
        
        stack_type = self.analyzer.analyze_stack_type(stack)
        self.assertEqual(stack_type, StackType.JSON)
    
    def test_analyze_raw_stack_type(self):
        """Test detection of raw stack type."""
        stack = {
            "node": ["test_node"],
            "composable": ["test_composable"]
        }
        
        stack_type = self.analyzer.analyze_stack_type(stack)
        self.assertEqual(stack_type, StackType.RAW)
    
    def test_analyze_legacy_stack_type(self):
        """Test detection of legacy stack type."""
        stack = {
            "launch_description_source": "test.launch.py",
            "on_start": "start_command",
            "on_kill": "kill_command"
        }
        
        stack_type = self.analyzer.analyze_stack_type(stack)
        self.assertEqual(stack_type, StackType.LEGACY)
    
    def test_analyze_legacy_archive_stack_type(self):
        """Test backward compatibility for legacy archive content type."""
        stack = {
            "metadata": {
                "content_type": "archive"
            }
        }
        
        stack_type = self.analyzer.analyze_stack_type(stack)
        self.assertEqual(stack_type, StackType.ARCHIVE)
    
    def test_analyze_legacy_json_stack_type(self):
        """Test backward compatibility for legacy JSON content type."""
        stack = {
            "metadata": {
                "content_type": "json"
            }
        }
        
        stack_type = self.analyzer.analyze_stack_type(stack)
        self.assertEqual(stack_type, StackType.JSON)
    
    def test_determine_archive_execution_requirements(self):
        """Test execution requirements for archive stack."""
        stack = {
            "metadata": {"content_type": "stack/archive"},
            "node": ["test_node"]
        }
        
        requirements = self.analyzer.determine_execution_requirements(stack)
        
        self.assertTrue(requirements.requires_provision)
        self.assertTrue(requirements.requires_launch)
        self.assertTrue(requirements.has_nodes)
        self.assertFalse(requirements.has_composables)
    
    def test_determine_json_execution_requirements(self):
        """Test execution requirements for JSON stack."""
        stack = {
            "metadata": {"content_type": "stack/json"},
            "composable": ["test_composable"]
        }
        
        requirements = self.analyzer.determine_execution_requirements(stack)
        
        self.assertFalse(requirements.requires_provision)
        self.assertTrue(requirements.requires_launch)
        self.assertFalse(requirements.has_nodes)
        self.assertTrue(requirements.has_composables)
    
    def test_handle_stack_request_event(self):
        """Test handling of stack request event."""
        # Create mock event handler to capture published events
        published_events = []
        
        def capture_event(event):
            published_events.append(event)
        
        self.event_bus.subscribe(EventType.STACK_ANALYZED, capture_event)
        
        # Create stack request event
        request_event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test",
            stack_name="test_stack",
            action="start",
            stack_payload={
                "metadata": {"content_type": "stack/archive"},
                "node": ["test_node"]
            }
        )
        
        # Handle the event
        self.analyzer.handle_stack_request(request_event)
        
        # Verify analyzed event was published
        self.assertEqual(len(published_events), 1)
        analyzed_event = published_events[0]
        self.assertEqual(analyzed_event.event_type, EventType.STACK_ANALYZED)
        self.assertEqual(analyzed_event.stack_name, "test_stack")
        self.assertEqual(analyzed_event.action, "start")
        self.assertEqual(analyzed_event.analysis_result["stack_type"], StackType.ARCHIVE.value)


class TestStackProcessor(unittest.TestCase):
    
    def setUp(self):
        self.event_bus = EventBus()
        self.logger = MagicMock()
        
        # Mock the stack parser
        with patch('composer.subsystems.stack_manager.create_stack_parser') as mock_parser:
            mock_parser.return_value = MagicMock()
            self.processor = StackProcessor(self.event_bus, self.logger)
    
    @patch('composer.subsystems.stack_manager.Stack')
    def test_merge_stacks(self, mock_stack_class):
        """Test stack merging functionality."""
        # Setup mock Stack behavior
        mock_current_stack = MagicMock()
        mock_next_stack = MagicMock()
        mock_merged = MagicMock()
        mock_merged.manifest = {"merged": "stack"}
        
        mock_stack_class.side_effect = [mock_current_stack, mock_next_stack]
        mock_current_stack.merge.return_value = mock_merged
        
        # Test merge
        current = {"current": "stack"}
        next_stack = {"next": "stack"}
        
        result = self.processor.merge_stacks(current, next_stack)
        
        # Verify Stack objects were created correctly
        mock_stack_class.assert_any_call(manifest=current)
        mock_stack_class.assert_any_call(manifest=next_stack)
        
        # Verify merge was called
        mock_current_stack.merge.assert_called_once_with(mock_next_stack)
        
        # Verify result
        self.assertEqual(result, {"merged": "stack"})
    
    def test_resolve_expressions_basic(self):
        """Test basic expression resolution."""
        stack_json = '{"command": "$(env HOME)/test"}'
        
        with patch('os.getenv', return_value='/home/user'):
            result = self.processor.resolve_expressions(stack_json)
            
            # Should resolve the environment variable
            self.assertIn('/home/user/test', result)
            self.assertNotIn('$(env HOME)', result)
    
    @patch('composer.subsystems.stack_manager.get_package_share_directory')
    def test_resolve_expressions_find(self, mock_get_package):
        """Test find expression resolution."""
        mock_get_package.return_value = '/opt/ros/jazzy/share/test_package'
        
        stack_json = '{"path": "$(find test_package)/config"}'
        
        result = self.processor.resolve_expressions(stack_json)
        
        # Should resolve the package path
        self.assertIn('/opt/ros/jazzy/share/test_package/config', result)
        self.assertNotIn('$(find test_package)', result)
    
    def test_parse_payload(self):
        """Test payload parsing."""
        payload = {"test": "data"}
        
        # Mock the stack parser
        self.processor.stack_parser.parse_payload.return_value = {"parsed": "data"}
        
        result = self.processor.parse_payload(payload)
        
        self.assertEqual(result, {"parsed": "data"})
        self.processor.stack_parser.parse_payload.assert_called_once_with(payload)


class TestStackStateManager(unittest.TestCase):
    
    def setUp(self):
        self.event_bus = EventBus()
        self.logger = MagicMock()
        self.state_manager = StackStateManager(self.event_bus, self.logger)
    
    def test_set_and_get_current_stack(self):
        """Test setting and getting current stack."""
        test_stack = {"test": "stack"}
        
        self.state_manager.set_current_stack(test_stack)
        result = self.state_manager.get_current_stack()
        
        self.assertEqual(result, test_stack)
    
    def test_set_and_get_next_stack(self):
        """Test setting and getting next stack."""
        test_stack = {"next": "stack"}
        
        self.state_manager.set_next_stack(test_stack)
        result = self.state_manager.get_next_stack()
        
        self.assertEqual(result, test_stack)
    
    def test_get_stack_transition_initial_deploy(self):
        """Test transition type determination for initial deploy."""
        self.state_manager.set_next_stack({"next": "stack"})
        
        transition = self.state_manager.get_stack_transition()
        
        self.assertEqual(transition.transition_type, "initial_deploy")
        self.assertIsNone(transition.current)
        self.assertEqual(transition.next, {"next": "stack"})
    
    def test_get_stack_transition_update(self):
        """Test transition type determination for update."""
        self.state_manager.set_current_stack({"current": "stack"})
        self.state_manager.set_next_stack({"next": "stack"})
        
        transition = self.state_manager.get_stack_transition()
        
        self.assertEqual(transition.transition_type, "update")
        self.assertEqual(transition.current, {"current": "stack"})
        self.assertEqual(transition.next, {"next": "stack"})


class TestStackManager(unittest.TestCase):
    
    def setUp(self):
        self.event_bus = EventBus()
        self.logger = MagicMock()
        
        # Mock the dependencies
        with patch('composer.subsystems.stack_manager.create_stack_parser'):
            self.stack_manager = StackManager(self.event_bus, self.logger)
    
    def test_initialization(self):
        """Test StackManager initialization."""
        self.assertIsNotNone(self.stack_manager.state_manager)
        self.assertIsNotNone(self.stack_manager.analyzer)
        self.assertIsNotNone(self.stack_manager.processor)
    
    def test_get_components(self):
        """Test getting individual components."""
        state_manager = self.stack_manager.get_state_manager()
        analyzer = self.stack_manager.get_analyzer()
        processor = self.stack_manager.get_processor()
        
        self.assertIsNotNone(state_manager)
        self.assertIsNotNone(analyzer)
        self.assertIsNotNone(processor)


if __name__ == '__main__':
    unittest.main()