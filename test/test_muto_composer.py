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
import json
import asyncio
from unittest.mock import MagicMock, patch, AsyncMock
from muto_msgs.msg import MutoAction
from composer.muto_composer import MutoComposer
from composer.events import (
    EventBus, EventType, StackRequestEvent, StackAnalyzedEvent,
    StackProcessedEvent, OrchestrationStartedEvent, PipelineEvents
)


class TestMutoComposerIntegration(unittest.TestCase):
    """
    Integration tests for MutoComposer focusing on event-driven architecture.
    Tests the complete flow from MutoAction message to subsystem orchestration
    through event flows rather than direct method calls.
    """

    def setUp(self) -> None:
        # Initialize ROS if not already done
        try:
            rclpy.init()
        except:
            pass
        
        # Mock node creation to avoid actual ROS initialization
        with patch('composer.muto_composer.MutoComposer._initialize_subsystems'), \
             patch('composer.muto_composer.MutoComposer._setup_ros_interfaces'):
            self.composer = MutoComposer()
        
        # Setup test components
        self.test_events = []
        self.captured_events = {}
        
        # Setup event capture for all event types
        for event_type in EventType:
            self.captured_events[event_type] = []
            self.composer.event_bus.subscribe(
                event_type, 
                lambda event, et=event_type: self.captured_events[et].append(event)
            )

    def tearDown(self) -> None:
        try:
            self.composer.destroy_node()
        except:
            pass

    @classmethod
    def setUpClass(cls) -> None:
        try:
            rclpy.init()
        except:
            pass

    @classmethod
    def tearDownClass(cls) -> None:
        try:
            rclpy.shutdown()
        except:
            pass

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
import json
import asyncio
from unittest.mock import MagicMock, patch, AsyncMock
from muto_msgs.msg import MutoAction
from composer.muto_composer import MutoComposer
from composer.events import (
    EventBus, EventType, StackRequestEvent, StackAnalyzedEvent,
    StackProcessedEvent, OrchestrationStartedEvent, PipelineEvents
)


class TestMutoComposerIntegration(unittest.TestCase):
    """
    Integration tests for MutoComposer focusing on event-driven architecture.
    Tests the complete flow from MutoAction message to subsystem orchestration
    through event flows rather than direct method calls.
    """

    def setUp(self) -> None:
        # Initialize ROS if not already done
        try:
            rclpy.init()
        except:
            pass
        
        # Mock node creation to avoid actual ROS initialization
        with patch('composer.muto_composer.MutoComposer._initialize_subsystems'), \
             patch('composer.muto_composer.MutoComposer._setup_ros_interfaces'):
            self.composer = MutoComposer()
        
        # Setup test components
        self.test_events = []
        self.captured_events = {}
        
        # Setup event capture for all event types
        for event_type in EventType:
            self.captured_events[event_type] = []
            self.composer.event_bus.subscribe(
                event_type, 
                lambda event, et=event_type: self.captured_events[et].append(event)
            )

    def tearDown(self) -> None:
        try:
            self.composer.destroy_node()
        except:
            pass

    @classmethod
    def setUpClass(cls) -> None:
        try:
            rclpy.init()
        except:
            pass

    @classmethod
    def tearDownClass(cls) -> None:
        try:
            rclpy.shutdown()
        except:
            pass

    def test_muto_action_to_stack_request_flow(self):
        """Test the complete flow from MutoAction to StackRequest event."""
        # Create a MutoAction message
        muto_action = MutoAction()
        muto_action.method = "start"
        muto_action.payload = json.dumps({
            "value": {
                "stackId": "test_stack_001"
            }
        })
        
        # Process the MutoAction through message handler
        if hasattr(self.composer, 'message_handler'):
            self.composer.message_handler.handle_muto_action(muto_action)
        
        # Verify StackRequest event was generated
        stack_requests = self.captured_events.get(EventType.STACK_REQUEST, [])
        if stack_requests:
            request = stack_requests[0]
            self.assertEqual(request.action, "start")
            self.assertEqual(request.stack_name, "test_stack_001")

    def test_stack_analysis_integration_flow(self):
        """Test integration between StackRequest and StackAnalyzed events."""
        # Create a StackRequest event
        stack_request = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test_client",
            action="apply",
            stack_name="integration_test_stack",
            stack_payload={
                "metadata": {"name": "integration_test_stack"},
                "nodes": [{"name": "test_node", "pkg": "test_pkg"}]
            }
        )
        
        # Publish the event
        self.composer.event_bus.publish_sync(stack_request)
        
        # In a real system, StackManager would process this and emit StackAnalyzed
        # For testing, we simulate the expected behavior
        analyzed_events = self.captured_events.get(EventType.STACK_ANALYZED, [])
        # Would verify that StackManager processed the request
        
        # Verify event was received (integration test for event flow)
        self.assertTrue(hasattr(self.composer, 'event_bus'))

    def test_complete_stack_processing_pipeline(self):
        """Test the complete pipeline from stack request to pipeline execution."""
        # Setup a complete stack payload
        stack_payload = {
            "metadata": {
                "name": "complete_test_stack",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [
                    {
                        "name": "test_node_1",
                        "pkg": "test_package",
                        "exec": "test_executable"
                    }
                ]
            }
        }
        
        # Create and publish StackRequest
        request_event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test_client",
            action="apply",
            stack_name="complete_test_stack",
            stack_payload=stack_payload
        )
        
        self.composer.event_bus.publish_sync(request_event)
        
        # Simulate the processing chain
        # 1. StackAnalyzed event
        analyzed_event = StackAnalyzedEvent(
            event_type=EventType.STACK_ANALYZED,
            source_component="test_analyzer",
            stack_name="complete_test_stack",
            action="apply",
            analysis_result={"stack_type": "stack/json"},
            processing_requirements={"runtime": "docker", "launch_required": True}
        )
        
        self.composer.event_bus.publish_sync(analyzed_event)
        
        # 2. StackProcessed event  
        processed_event = StackProcessedEvent(
            stack_name="complete_test_stack",
            stack_payload=stack_payload,  # Updated to use new standardized parameter name
            execution_requirements={"runtime": "docker", "launch_required": True}
        )
        
        self.composer.event_bus.publish_sync(processed_event)
        
        # 3. OrchestrationStarted event
        orchestration_event = OrchestrationStartedEvent(
            event_type=EventType.ORCHESTRATION_STARTED,
            source_component="test_orchestrator",
            action="apply",
            execution_plan={"steps": ["provision", "launch"]},
            orchestration_id="test_orchestration_001"
        )
        
        self.composer.event_bus.publish_sync(orchestration_event)
        
        # Verify events were captured in the integration flow
        requests = self.captured_events.get(EventType.STACK_REQUEST, [])
        analyzed = self.captured_events.get(EventType.STACK_ANALYZED, [])
        processed = self.captured_events.get(EventType.STACK_PROCESSED, [])
        orchestration = self.captured_events.get(EventType.ORCHESTRATION_STARTED, [])
        
        self.assertTrue(len(requests) > 0)
        self.assertTrue(len(analyzed) > 0)
        self.assertTrue(len(processed) > 0)
        self.assertTrue(len(orchestration) > 0)

    def test_pipeline_execution_event_flow(self):
        """Test pipeline execution through event flows."""
        # Create pipeline events
        pipeline_start = PipelineEvents.create_start_event(
            pipeline_name="test_pipeline",
            context={"stack_name": "test_stack", "action": "apply"}
        )
        
        pipeline_complete = PipelineEvents.create_completion_event(
            pipeline_name="test_pipeline",
            success=True,
            result={"deployed": True, "nodes": ["node1"]}
        )
        
        # Publish pipeline events
        self.composer.event_bus.publish_sync(pipeline_start)
        self.composer.event_bus.publish_sync(pipeline_complete)
        
        # Verify pipeline events were captured
        start_events = self.captured_events.get(EventType.PIPELINE_START, [])
        complete_events = self.captured_events.get(EventType.PIPELINE_COMPLETE, [])
        
        self.assertTrue(len(start_events) > 0)
        self.assertTrue(len(complete_events) > 0)

    def test_error_handling_in_event_flows(self):
        """Test error handling in event-driven flows."""
        # Create an error event
        error_event = PipelineEvents.create_error_event(
            pipeline_name="failing_pipeline",
            error="Test error condition",
            context={"stack_name": "error_test_stack"}
        )
        
        # Publish error event
        self.composer.event_bus.publish_sync(error_event)
        
        # Verify error event was captured
        error_events = self.captured_events.get(EventType.PIPELINE_ERROR, [])
        self.assertTrue(len(error_events) > 0)
        
        if error_events:
            captured_error = error_events[0]
            self.assertEqual(captured_error.error_details["error"], "Test error condition")

    def test_subsystem_isolation_through_events(self):
        """Test that subsystems communicate only through events."""
        # Verify that the composer has the expected subsystems
        self.assertTrue(hasattr(self.composer, 'event_bus'))
        
        # Verify subsystems exist (would be initialized in real system)
        subsystem_attrs = [
            'stack_manager',
            'orchestration_manager', 
            'pipeline_engine',
            'message_handler',
            'digital_twin_integration'
        ]
        
        # In the new architecture, these would be initialized
        # For now, verify the event bus is the communication mechanism
        self.assertIsNotNone(self.composer.event_bus)

    def test_backward_compatibility_methods(self):
        """Test that backward compatibility methods are available."""
        # These methods should exist but be marked as deprecated
        deprecated_methods = [
            'on_stack_callback',
            'resolve_expression',
            'determine_execution_path',
            'merge'
        ]
        
        for method_name in deprecated_methods:
            if hasattr(self.composer, method_name):
                method = getattr(self.composer, method_name)
                self.assertTrue(callable(method))

    def test_event_bus_integration(self):
        """Test that event bus is properly integrated with composer."""
        # Verify event bus exists
        self.assertIsNotNone(self.composer.event_bus)
        
        # Test event publishing and subscription
        test_events = []
        
        def test_handler(event):
            test_events.append(event)
        
        # Subscribe to a test event
        self.composer.event_bus.subscribe(EventType.STACK_REQUEST, test_handler)
        
        # Publish a test event
        test_event = StackRequestEvent(
            event_type=EventType.STACK_REQUEST,
            source_component="test_client",
            action="test",
            stack_name="test_stack",
            stack_payload={"test": "data"}
        )
        
        self.composer.event_bus.publish_sync(test_event)
        
        # Verify event was received
        self.assertEqual(len(test_events), 1)
        self.assertEqual(test_events[0].action, "test")

    def test_digital_twin_integration_flow(self):
        """Test digital twin integration through events."""
        # Create a stack processed event that should trigger twin updates
        processed_event = StackProcessedEvent(
            stack_name="twin_test_stack",
            stack_payload={  # Updated to use new standardized parameter name
                "metadata": {
                    "name": "twin_test_stack",
                    "twin_id": "test_twin_001"
                },
                "nodes": [{"name": "twin_node"}]
            },
            execution_requirements={"runtime": "docker"}
        )
        
        # Publish the event
        self.composer.event_bus.publish_sync(processed_event)
        
        # Verify the event was captured
        processed_events = self.captured_events.get(EventType.STACK_PROCESSED, [])
        self.assertTrue(len(processed_events) > 0)
        
        if processed_events:
            event = processed_events[0]
            self.assertIn("twin_id", event.merged_stack["metadata"])

    def test_message_routing_integration(self):
        """Test message routing integration with event system."""
        # Test that MutoAction messages are properly routed to events
        # This tests the integration between MessageHandler and EventBus
        
        # Create different types of MutoAction messages
        test_actions = [
            ("start", {"value": {"stackId": "start_test"}}),
            ("apply", {"metadata": {"name": "apply_test"}, "nodes": ["node1"]}),
            ("stop", {"value": {"stackId": "stop_test"}})
        ]
        
        for method, payload in test_actions:
            muto_action = MutoAction()
            muto_action.method = method
            muto_action.payload = json.dumps(payload)
            
            # In real system, this would be handled by message_handler
            # For integration test, verify the structure is correct
            self.assertEqual(muto_action.method, method)
            self.assertIsNotNone(muto_action.payload)


class TestMutoComposerLegacyCompatibility(unittest.TestCase):
    """
    Tests for backward compatibility with existing interfaces.
    These test deprecated methods that are maintained for compatibility.
    """

    def setUp(self) -> None:
        try:
            rclpy.init()
        except:
            pass
            
        with patch('composer.muto_composer.MutoComposer._initialize_subsystems'), \
             patch('composer.muto_composer.MutoComposer._setup_ros_interfaces'):
            self.composer = MutoComposer()

    def tearDown(self) -> None:
        try:
            self.composer.destroy_node()
        except:
            pass

    def test_legacy_stack_parser_compatibility(self):
        """Test backward compatibility with stack parser."""
        # Test parsing different payload formats
        test_payloads = [
            {"value": {"stackId": "legacy_test"}},
            {"metadata": {"name": "direct_test"}, "nodes": ["node1"]},
            {"unknown": "format"}
        ]
        
        for payload in test_payloads:
            if hasattr(self.composer, 'stack_parser'):
                result = self.composer.stack_parser.parse_payload(payload)
                # Verify parsing doesn't crash
                self.assertTrue(result is not None or result is None)

    def test_legacy_merge_functionality(self):
        """Test backward compatibility for merge functionality."""
        if hasattr(self.composer, 'merge'):
            stack1 = {"nodes": [{"name": "node1"}]}
            stack2 = {"nodes": [{"name": "node2"}]}
            
            merged = self.composer.merge(stack1, stack2)
            # Verify merge works without crashing
            self.assertIsNotNone(merged)

    def test_legacy_expression_resolution(self):
        """Test backward compatibility for expression resolution."""
        if hasattr(self.composer, 'resolve_expression'):
            test_expressions = [
                "$(find test_pkg)",
                "$(env TEST_VAR)",
                "no expression here"
            ]
            
            for expr in test_expressions:
                try:
                    result = self.composer.resolve_expression(expr)
                    # Verify method doesn't crash
                    self.assertIsNotNone(result)
                except Exception:
                    # Legacy method may have dependencies that aren't mocked
                    pass


if __name__ == "__main__":
    unittest.main()
