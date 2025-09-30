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
import rclpy
from muto_msgs.msg import MutoAction
from composer.events import EventBus, EventType, StackRequestEvent
from composer.subsystems.message_handler import MessageHandler, MessageRouter
from composer.subsystems.stack_manager import StackType


class TestMessageRouter(unittest.TestCase):
    
    def setUp(self):
        self.event_bus = EventBus()
        self.logger = MagicMock()
        self.router = MessageRouter(self.event_bus, self.logger)
    
    def test_route_muto_action_with_value_key(self):
        """Test routing MutoAction with value key."""
        # Setup event capture
        routed_events = []
        
        def capture_event(event):
            routed_events.append(event)
        
        self.event_bus.subscribe(EventType.STACK_REQUEST, capture_event)
        
        # Create MutoAction message
        muto_action = MutoAction()
        muto_action.method = "start"
        muto_action.payload = '{"value": {"stackId": "test_stack"}}'
        
        # Route the message
        self.router.route_muto_action(muto_action)
        
        # Verify event was published
        self.assertEqual(len(routed_events), 1)
        event = routed_events[0]
        self.assertEqual(event.action, "start")
        self.assertEqual(event.stack_name, "test_stack")
    
    def test_route_muto_action_direct_payload(self):
        """Test routing MutoAction with direct payload."""
        # Setup event capture
        routed_events = []
        
        def capture_event(event):
            routed_events.append(event)
        
        self.event_bus.subscribe(EventType.STACK_REQUEST, capture_event)
        
        # Create MutoAction message with direct payload
        muto_action = MutoAction()
        muto_action.method = "apply"
        muto_action.payload = '{"node": ["test_node"], "metadata": {"name": "direct_stack"}}'
        
        # Route the message
        self.router.route_muto_action(muto_action)
        
        # Verify event was published
        self.assertEqual(len(routed_events), 1)
        event = routed_events[0]
        self.assertEqual(event.action, "apply")
        self.assertEqual(event.stack_name, "direct_stack")
        self.assertIn("node", event.stack_payload)
    
    def test_route_muto_action_invalid_json(self):
        """Test routing MutoAction with invalid JSON."""
        muto_action = MutoAction()
        muto_action.method = "start"
        muto_action.payload = 'invalid json'
        
        # Should not raise exception, should log error
        self.router.route_muto_action(muto_action)
        
        # Verify error was logged
        self.logger.error.assert_called()
    
    def test_extract_stack_name_from_value_key(self):
        """Test stack name extraction from value key."""
        payload = {"value": {"stackId": "test_stack_123"}}
        
        stack_name = self.router._extract_stack_name(payload, "test_namespace:test_device")
        
        self.assertEqual(stack_name, "test_stack_123")
    
    def test_extract_stack_name_from_metadata(self):
        """Test stack name extraction from metadata."""
        payload = {"metadata": {"name": "metadata_stack"}}
        
        stack_name = self.router._extract_stack_name(payload, "test_namespace:test_device")
        
        self.assertEqual(stack_name, "metadata_stack")
    
    def test_extract_stack_name_fallback(self):
        """Test stack name extraction fallback to default."""
        payload = {"some": "data"}
        default_name = "test_namespace:test_device"
        
        stack_name = self.router._extract_stack_name(payload, default_name)
        
        self.assertEqual(stack_name, default_name)


class TestMessageHandler(unittest.TestCase):
    
    def setUp(self):
        # Initialize ROS if not already done
        try:
            rclpy.init()
        except:
            pass
        
        # Create a mock node
        self.mock_node = MagicMock()
        self.mock_node.get_logger.return_value = MagicMock()
        self.mock_node.get_parameter.return_value.get_parameter_value.return_value.string_value = "test_value"
        
        self.event_bus = EventBus()
        self.logger = MagicMock()
        
        self.message_handler = MessageHandler(self.mock_node, self.event_bus, self.logger)
    
    def tearDown(self):
        try:
            rclpy.shutdown()
        except:
            pass
    
    def test_initialization(self):
        """Test MessageHandler initialization."""
        self.assertIsNotNone(self.message_handler.router)
        self.assertIsNotNone(self.message_handler.publisher_manager)
        self.assertIsNotNone(self.message_handler.service_client_manager)
    
    def test_handle_muto_action(self):
        """Test handling MutoAction message."""
        # Setup event capture
        handled_events = []
        
        def capture_event(event):
            handled_events.append(event)
        
        self.event_bus.subscribe(EventType.STACK_REQUEST, capture_event)
        
        # Create MutoAction
        muto_action = MutoAction()
        muto_action.method = "start"
        muto_action.payload = '{"value": {"stackId": "test_stack"}}'
        
        # Handle the message
        self.message_handler.handle_muto_action(muto_action)
        
        # Verify event was generated
        self.assertEqual(len(handled_events), 1)
        event = handled_events[0]
        self.assertEqual(event.action, "start")
        self.assertEqual(event.stack_name, "test_stack")
    
    def test_get_components(self):
        """Test getting individual components."""
        router = self.message_handler.get_router()
        publisher_manager = self.message_handler.get_publisher_manager()
        service_client_manager = self.message_handler.get_service_client_manager()
        
        self.assertIsNotNone(router)
        self.assertIsNotNone(publisher_manager)
        self.assertIsNotNone(service_client_manager)
    
    def test_integration_with_composer_flow(self):
        """Test integration with overall composer flow."""
        # This would test the complete flow from MutoAction to events
        # Setup multiple event captures to verify the full chain
        
        stack_requests = []
        
        def capture_stack_request(event):
            stack_requests.append(event)
        
        self.event_bus.subscribe(EventType.STACK_REQUEST, capture_stack_request)
        
        # Simulate receiving a complex MutoAction
        muto_action = MutoAction()
        muto_action.method = "apply"
        muto_action.payload = '''
        {
            "metadata": {
                "name": "integration_test_stack",
                "content_type": "stack/json"
            },
            "node": ["test_node_1", "test_node_2"],
            "launch": {
                "test_param": "value"
            }
        }
        '''
        
        # Handle the message
        self.message_handler.handle_muto_action(muto_action)
        
        # Verify the complete event was created correctly
        self.assertEqual(len(stack_requests), 1)
        request = stack_requests[0]
        
        self.assertEqual(request.action, "apply")
        self.assertEqual(request.stack_name, "integration_test_stack")
        self.assertIn("node", request.stack_payload)
        self.assertIn("launch", request.stack_payload)
        self.assertEqual(request.stack_payload["metadata"]["content_type"], StackType.JSON.value)


if __name__ == '__main__':
    unittest.main()