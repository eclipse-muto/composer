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
Validation tests for the refactored event-driven architecture.
Tests that the new modular design works correctly without dependencies on deprecated functionality.
"""

import unittest
from unittest.mock import MagicMock, patch
from composer.events import EventBus, EventType, StackRequestEvent, StackAnalyzedEvent


class TestArchitectureValidation(unittest.TestCase):
    """Validate the new event-driven architecture."""
    
    def setUp(self):
        self.event_bus = EventBus()
        
    def test_event_bus_basic_functionality(self):
        """Test that EventBus works for basic publish/subscribe."""
        events_received = []
        
        def test_handler(event):
            events_received.append(event)
        
        # Subscribe to stack request events
        self.event_bus.subscribe(EventType.STACK_REQUEST, test_handler)
        
        # Create and publish an event
        event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test",
            stack_name="test_stack",
            action="start"
        )
        
        self.event_bus.publish_sync(event)
        
        # Verify event was received
        self.assertEqual(len(events_received), 1)
        self.assertEqual(events_received[0].stack_name, "test_stack")
        self.assertEqual(events_received[0].action, "start")
    
    def test_multiple_event_types(self):
        """Test handling multiple event types."""
        stack_requests = []
        stack_analyzed = []
        
        def handle_request(event):
            stack_requests.append(event)
        
        def handle_analyzed(event):
            stack_analyzed.append(event)
        
        # Subscribe to different event types
        self.event_bus.subscribe(EventType.STACK_REQUEST, handle_request)
        self.event_bus.subscribe(EventType.STACK_ANALYZED, handle_analyzed)
        
        # Publish different types of events
        request_event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test",
            stack_name="test_stack",
            action="apply"
        )
        
        analyzed_event = StackAnalyzedEvent(
            event_type=EventType.STACK_ANALYZED,
            source_component="analyzer",
            stack_name="test_stack",
            action="apply"
        )
        
        self.event_bus.publish_sync(request_event)
        self.event_bus.publish_sync(analyzed_event)
        
        # Verify each handler only received its event type
        self.assertEqual(len(stack_requests), 1)
        self.assertEqual(len(stack_analyzed), 1)
        self.assertEqual(stack_requests[0].action, "apply")
        self.assertEqual(stack_analyzed[0].action, "apply")
    
    def test_event_isolation(self):
        """Test that events are properly isolated between handlers."""
        handler1_events = []
        handler2_events = []
        
        def handler1(event):
            handler1_events.append(event)
        
        def handler2(event):
            handler2_events.append(event)
        
        # Subscribe both handlers to same event type
        self.event_bus.subscribe(EventType.STACK_REQUEST, handler1)
        self.event_bus.subscribe(EventType.STACK_REQUEST, handler2)
        
        # Publish event
        event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test",
            stack_name="isolation_test",
            action="start"
        )
        
        self.event_bus.publish_sync(event)
        
        # Both handlers should receive the event
        self.assertEqual(len(handler1_events), 1)
        self.assertEqual(len(handler2_events), 1)
        
        # But they should be independent
        self.assertEqual(handler1_events[0].stack_name, "isolation_test")
        self.assertEqual(handler2_events[0].stack_name, "isolation_test")
    
    def test_subsystem_communication_pattern(self):
        """Test the intended subsystem communication pattern through events."""
        # This simulates how subsystems should communicate:
        # MessageHandler -> StackManager -> OrchestrationManager -> PipelineEngine
        
        communication_flow = []
        
        def message_handler_simulator(event):
            # Simulate MessageHandler receiving MutoAction and creating StackRequest
            if event.event_type == EventType.STACK_REQUEST:
                communication_flow.append("MessageHandler->StackRequest")
                
                # MessageHandler would publish a StackRequest event
                # StackManager would subscribe to this and emit StackAnalyzed
                analyzed_event = StackAnalyzedEvent(
                    event_type=EventType.STACK_ANALYZED,
                    source_component="stack_manager",
                    stack_name=event.stack_name,
                    action=event.action
                )
                self.event_bus.publish_sync(analyzed_event)
        
        def stack_manager_simulator(event):
            if event.event_type == EventType.STACK_ANALYZED:
                communication_flow.append("StackManager->StackAnalyzed")
                
                # Would emit OrchestrationStarted, but we'll just track the flow
                communication_flow.append("StackManager->OrchestrationRequest")
        
        # Set up the communication chain
        self.event_bus.subscribe(EventType.STACK_REQUEST, message_handler_simulator)
        self.event_bus.subscribe(EventType.STACK_ANALYZED, stack_manager_simulator)
        
        # Start the flow with a StackRequest
        initial_event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test_client",
            stack_name="communication_test",
            action="apply"
        )
        
        self.event_bus.publish_sync(initial_event)
        
        # Verify the communication flow
        expected_flow = [
            "MessageHandler->StackRequest",
            "StackManager->StackAnalyzed", 
            "StackManager->OrchestrationRequest"
        ]
        
        self.assertEqual(communication_flow, expected_flow)
    
    def test_event_metadata_preservation(self):
        """Test that event metadata is preserved through the system."""
        received_events = []
        
        def metadata_handler(event):
            received_events.append(event)
        
        self.event_bus.subscribe(EventType.STACK_REQUEST, metadata_handler)
        
        # Create event with metadata
        event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test",
            stack_name="metadata_test",
            action="start",
            correlation_id="test_correlation_123",
            metadata={
                "client_id": "test_client",
                "priority": "high",
                "deployment_target": "edge_device_001"
            }
        )
        
        self.event_bus.publish_sync(event)
        
        # Verify metadata is preserved
        received_event = received_events[0]
        self.assertEqual(received_event.correlation_id, "test_correlation_123")
        self.assertEqual(received_event.metadata["client_id"], "test_client")
        self.assertEqual(received_event.metadata["priority"], "high")
        self.assertEqual(received_event.metadata["deployment_target"], "edge_device_001")


class TestStackRequestEventValidation(unittest.TestCase):
    """Validate StackRequest event functionality."""
    
    def test_stack_request_creation(self):
        """Test creating StackRequest events with different payloads."""
        # Test with JSON stack
        json_stack_event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="message_handler",
            stack_name="json_test_stack",
            action="apply",
            stack_payload={
                "metadata": {"name": "json_test_stack", "content_type": "stack/json"},
                "launch": {"node": [{"name": "test_node", "pkg": "test_pkg"}]}
            }
        )
        
        self.assertEqual(json_stack_event.stack_name, "json_test_stack")
        self.assertEqual(json_stack_event.action, "apply")
        self.assertIn("metadata", json_stack_event.stack_payload)
        self.assertEqual(
            json_stack_event.stack_payload["metadata"]["content_type"], 
            "stack/json"
        )
        
        # Test with archive stack
        archive_stack_event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="message_handler",
            stack_name="archive_test_stack",
            action="deploy",
            stack_payload={
                "metadata": {"name": "archive_test_stack", "content_type": "stack/archive"},
                "launch": {
                    "data": "base64_encoded_archive_data",
                    "properties": {"launch_file": "launch/test.launch.py"}
                }
            }
        )
        
        self.assertEqual(archive_stack_event.stack_name, "archive_test_stack")
        self.assertEqual(archive_stack_event.action, "deploy")
        self.assertEqual(
            archive_stack_event.stack_payload["metadata"]["content_type"],
            "stack/archive"
        )
    
    def test_stack_analyzed_event_creation(self):
        """Test creating StackAnalyzed events."""
        analyzed_event = StackAnalyzedEvent(
            event_type=EventType.STACK_ANALYZED,
            source_component="stack_analyzer",
            stack_name="analyzed_stack",
            action="apply",
            analysis_result={
                "stack_type": "json",
                "complexity": "medium",
                "estimated_resources": {"cpu": "0.5", "memory": "512Mi"}
            },
            processing_requirements={
                "requires_provision": True,
                "requires_launch": True,
                "execution_order": ["provision", "launch"]
            }
        )
        
        self.assertEqual(analyzed_event.stack_name, "analyzed_stack")
        self.assertEqual(analyzed_event.analysis_result["stack_type"], "json")
        self.assertTrue(analyzed_event.processing_requirements["requires_provision"])
        self.assertEqual(
            analyzed_event.processing_requirements["execution_order"],
            ["provision", "launch"]
        )


if __name__ == "__main__":
    unittest.main()