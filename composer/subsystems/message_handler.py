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
Message handling subsystem for the refactored Muto Composer.
Manages all ROS 2 communication including topics, services, and publishers.
"""

import json
from typing import Dict, Any, Optional
from rclpy.node import Node
from std_msgs.msg import String
from muto_msgs.msg import MutoAction
from muto_msgs.srv import CoreTwin
from composer.events import EventBus, StackRequestEvent, EventType


class MessageRouter:
    """Routes incoming messages to appropriate handlers via events."""
    
    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
    
    def route_muto_action(self, action: MutoAction) -> None:
        """Route MutoAction to orchestration manager via events."""
        try:
            payload = json.loads(action.payload)
            stack_name = self._extract_stack_name(payload, f"unknown:{action.method}")
            
            event = StackRequestEvent(
                event_type=EventType.STACK_REQUEST,
                source_component="message_router",
                stack_name=stack_name,
                action=action.method,
                stack_payload=payload
            )
            
            if self.logger:
                self.logger.info(f"Routing {action.method} action via event system")
            
            self.event_bus.publish_sync(event)
            
        except json.JSONDecodeError as e:
            if self.logger:
                self.logger.error(f"Failed to parse MutoAction payload: {e}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error routing MutoAction: {e}")
    
    def _extract_stack_name(self, payload: Dict[str, Any], default_name: str) -> str:
        """Extract stack name from payload."""
        # Try to extract from value key
        if 'value' in payload and 'stackId' in payload['value']:
            return payload['value']['stackId']
        
        # Try to extract from metadata
        if 'metadata' in payload and 'name' in payload['metadata']:
            return payload['metadata']['name']
        
        # Return default if not found
        return default_name


class PublisherManager:
    """Manages all outbound publishing with consolidated publishers."""
    
    def __init__(self, node: Node):
        self.node = node
        # Consolidated publisher instead of multiple deprecated ones
        self.stack_state_pub = node.create_publisher(String, "stack_state", 10)
        self.logger = node.get_logger()
    
    def publish_stack_state(self, stack_data: Dict[str, Any], state_type: str = "current") -> None:
        """Publish consolidated stack state information."""
        try:
            # Create consolidated state message
            state_message = {
                "type": state_type,
                "timestamp": str(self.node.get_clock().now().to_msg()),
                "data": stack_data
            }
            
            msg = String()
            msg.data = json.dumps(state_message)
            self.stack_state_pub.publish(msg)
            
            self.logger.debug(f"Published {state_type} stack state")
            
        except Exception as e:
            self.logger.error(f"Error publishing stack state: {e}")


class ServiceClientManager:
    """Manages service client connections and calls."""
    
    def __init__(self, node: Node, core_twin_node_name: str = "core_twin"):
        self.node = node
        self.logger = node.get_logger()
        
        # Initialize service clients
        self.get_stack_client = node.create_client(
            CoreTwin, 
            f"{core_twin_node_name}/get_stack_definition"
        )
        self.set_stack_client = node.create_client(
            CoreTwin, 
            f"{core_twin_node_name}/set_current_stack"
        )
    
    async def get_stack_definition(self, stack_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve stack definition from twin service."""
        try:
            request = CoreTwin.Request()
            request.input = stack_id
            
            if not self.get_stack_client.wait_for_service(timeout_sec=5.0):
                self.logger.error("CoreTwin get_stack_definition service not available")
                return None
            
            future = self.get_stack_client.call_async(request)
            # Note: In real implementation, this would be properly awaited
            # For now, we'll use the existing callback pattern
            
            return {}  # Placeholder
            
        except Exception as e:
            self.logger.error(f"Error calling get_stack_definition: {e}")
            return None
    
    async def set_current_stack(self, stack_id: str) -> bool:
        """Update current stack in twin service."""
        try:
            request = CoreTwin.Request()
            request.input = stack_id
            
            if not self.set_stack_client.wait_for_service(timeout_sec=5.0):
                self.logger.error("CoreTwin set_current_stack service not available")
                return False
            
            future = self.set_stack_client.call_async(request)
            # Note: In real implementation, this would be properly awaited
            
            return True  # Placeholder
            
        except Exception as e:
            self.logger.error(f"Error calling set_current_stack: {e}")
            return False


class MessageHandler:
    """Main message handling subsystem coordinator."""
    
    def __init__(self, node: Node, event_bus: EventBus, core_twin_node_name: str = "core_twin"):
        self.node = node
        self.event_bus = event_bus
        self.logger = node.get_logger()
        
        # Initialize components
        self.router = MessageRouter(event_bus, self.logger)
        self.publisher_manager = PublisherManager(node)
        self.service_manager = ServiceClientManager(node, core_twin_node_name)
        # Add alias for compatibility
        self.service_client_manager = self.service_manager
        
        # Set up subscribers
        self._setup_subscribers()
        
        self.logger.info("MessageHandler subsystem initialized")
    
    def _setup_subscribers(self):
        """Set up ROS 2 subscribers."""
        # Get stack topic from parameters
        stack_topic = self.node.get_parameter("stack_topic").get_parameter_value().string_value
        
        # Subscribe to MutoAction messages
        self.node.create_subscription(
            MutoAction,
            stack_topic,
            self._muto_action_callback,
            10
        )
        
        self.logger.info(f"Subscribed to {stack_topic} for MutoAction messages")
    
    def _muto_action_callback(self, msg: MutoAction):
        """Callback for MutoAction messages."""
        try:
            self.logger.info(f"Received MutoAction: {msg.method}")
            self.router.route_muto_action(msg)
        except Exception as e:
            self.logger.error(f"Error in MutoAction callback: {e}")
    
    def publish_stack_state(self, stack_data: Dict[str, Any], state_type: str = "current"):
        """Publish stack state through publisher manager."""
        self.publisher_manager.publish_stack_state(stack_data, state_type)
    
    def get_service_manager(self) -> ServiceClientManager:
        """Get service client manager for external use."""
        return self.service_manager
    
    def handle_muto_action(self, muto_action: MutoAction):
        """Handle MutoAction message."""
        self.router.route_muto_action(muto_action)
    
    def get_router(self) -> MessageRouter:
        """Get message router."""
        return self.router
    
    def get_publisher_manager(self) -> PublisherManager:
        """Get publisher manager."""
        return self.publisher_manager
    
    def get_service_client_manager(self) -> ServiceClientManager:
        """Get service client manager (alias for compatibility)."""
        return self.service_manager