#!/usr/bin/env python3

"""
Test for stack analyzed processing flow.
Tests that handle_stack_analyzed properly processes and emits results.
"""

import unittest
from unittest.mock import MagicMock, patch
from composer.events import EventBus, EventType, StackAnalyzedEvent, StackProcessedEvent
from composer.subsystems.stack_manager import StackProcessor


class TestStackAnalyzedProcessing(unittest.TestCase):
    
    def setUp(self):
        self.event_bus = MagicMock()
        self.logger = MagicMock()
        
        # Mock the stack parser
        with patch('composer.subsystems.stack_manager.create_stack_parser') as mock_parser:
            mock_parser.return_value = MagicMock()
            self.processor = StackProcessor(self.event_bus, self.logger)
    
    def test_handle_stack_analyzed_with_merge_processing(self):
        """Test that merge processing is applied and results flow through."""
        
        # Create test event with merge processing requirement
        test_event = StackAnalyzedEvent(
            event_type=EventType.STACK_ANALYZED,
            source_component="test",
            correlation_id="test-123",
            stack_name="test_stack",
            action="start",
            stack_payload={"test": "original"},
            processing_requirements={"merge_manifests": True}
        )
        
        # Mock the merge_stacks method to return known result
        with patch.object(self.processor, 'merge_stacks') as mock_merge:
            mock_merge.return_value = {"test": "merged", "merged": True}
            
            # Call the method
            self.processor.handle_stack_analyzed(test_event)
            
            # Verify merge was called
            mock_merge.assert_called_once_with({}, {"test": "original"})
            
            # Verify processed event was published
            self.event_bus.publish_async.assert_called_once()
            published_event = self.event_bus.publish_async.call_args[0][0]
            
            # Verify the published event is correct
            self.assertIsInstance(published_event, StackProcessedEvent)
            self.assertEqual(published_event.stack_payload, {"test": "merged", "merged": True})
            self.assertEqual(published_event.original_payload, {"test": "original"})
            self.assertEqual(published_event.processing_applied, ["merge_manifests"])
    
    def test_handle_stack_analyzed_with_expression_processing(self):
        """Test that expression processing is applied and results flow through."""
        
        # Create test event with expression processing requirement
        test_event = StackAnalyzedEvent(
            event_type=EventType.STACK_ANALYZED,
            source_component="test",
            correlation_id="test-456",
            stack_name="test_stack",
            action="start",
            stack_payload={"command": "$(env HOME)/test"},
            processing_requirements={"resolve_expressions": True}
        )
        
        # Mock the resolve_expressions method to return known result
        with patch.object(self.processor, 'resolve_expressions') as mock_resolve:
            mock_resolve.return_value = '{"command": "/home/user/test", "resolved": true}'
            
            # Call the method
            self.processor.handle_stack_analyzed(test_event)
            
            # Verify resolve_expressions was called
            mock_resolve.assert_called_once_with('{"command": "$(env HOME)/test"}')
            
            # Verify processed event was published
            self.event_bus.publish_async.assert_called_once()
            published_event = self.event_bus.publish_async.call_args[0][0]
            
            # Verify the published event is correct
            self.assertIsInstance(published_event, StackProcessedEvent)
            self.assertEqual(published_event.stack_payload, {"command": "/home/user/test", "resolved": True})
            self.assertEqual(published_event.original_payload, {"command": "$(env HOME)/test"})
            self.assertEqual(published_event.processing_applied, ["resolve_expressions"])
    
    def test_handle_stack_analyzed_with_both_processing(self):
        """Test that both merge and expression processing are applied sequentially."""
        
        # Create test event with both processing requirements
        test_event = StackAnalyzedEvent(
            event_type=EventType.STACK_ANALYZED,
            source_component="test",
            correlation_id="test-789",
            stack_name="test_stack",
            action="start",
            stack_payload={"command": "$(env HOME)/test"},
            processing_requirements={"merge_manifests": True, "resolve_expressions": True}
        )
        
        # Mock both processing methods
        with patch.object(self.processor, 'merge_stacks') as mock_merge, \
             patch.object(self.processor, 'resolve_expressions') as mock_resolve:
            
            mock_merge.return_value = {"command": "$(env HOME)/test", "merged": True}
            mock_resolve.return_value = '{"command": "/home/user/test", "merged": true, "resolved": true}'
            
            # Call the method
            self.processor.handle_stack_analyzed(test_event)
            
            # Verify both processing methods were called in order
            mock_merge.assert_called_once_with({}, {"command": "$(env HOME)/test"})
            mock_resolve.assert_called_once_with('{"command": "$(env HOME)/test", "merged": true}')
            
            # Verify processed event was published
            self.event_bus.publish_async.assert_called_once()
            published_event = self.event_bus.publish_async.call_args[0][0]
            
            # Verify the published event has both processing results
            self.assertIsInstance(published_event, StackProcessedEvent)
            expected_payload = {"command": "/home/user/test", "merged": True, "resolved": True}
            self.assertEqual(published_event.stack_payload, expected_payload)
            self.assertEqual(published_event.processing_applied, ["merge_manifests", "resolve_expressions"])
    
    def test_handle_stack_analyzed_no_processing_required(self):
        """Test that no processing event is emitted when no processing is required."""
        
        # Create test event with no processing requirements
        test_event = StackAnalyzedEvent(
            event_type=EventType.STACK_ANALYZED,
            source_component="test",
            correlation_id="test-000",
            stack_name="test_stack",
            action="start",
            stack_payload={"test": "data"},
            processing_requirements={}
        )
        
        # Call the method
        self.processor.handle_stack_analyzed(test_event)
        
        # Verify no processing event was published
        self.event_bus.publish_async.assert_not_called()


if __name__ == "__main__":
    unittest.main()