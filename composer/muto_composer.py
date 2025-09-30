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
Refactored Muto Composer using modular, event-driven architecture.
Coordinates subsystems to handle stack deployment orchestration.
"""

import os
import json
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from muto_msgs.msg import MutoAction
from composer.events import EventBus, EventType, StackRequestEvent
from composer.subsystems.message_handler import MessageHandler
from composer.subsystems.stack_manager import StackManager
from composer.subsystems.orchestration_manager import OrchestrationManager
from composer.subsystems.pipeline_engine import PipelineEngine
from composer.subsystems.digital_twin_integration import DigitalTwinIntegration

# Legacy imports for test compatibility
from composer.workflow.pipeline import Pipeline
from composer.utils.stack_parser import create_stack_parser


class MutoComposer(Node):
    """
    Refactored Muto Composer using modular, event-driven architecture.
    Coordinates subsystems to handle stack deployment orchestration.
    """
    
    def __init__(self):
        super().__init__("muto_composer")
        
        # Initialize configuration parameters
        self.declare_parameter("stack_topic", "stack")
        self.declare_parameter("twin_url", "sandbox.composiv.ai")
        self.declare_parameter("namespace", "org.eclipse.muto.sandbox")
        self.declare_parameter("name", "example-01")
        
        # Extract parameter values
        self.twin_url = self.get_parameter("twin_url").get_parameter_value().string_value
        self.twin_namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.name = self.get_parameter("name").get_parameter_value().string_value
        self.next_stack_topic = self.get_parameter("stack_topic").get_parameter_value().string_value
        
        # Initialize event bus for subsystem communication
        self.event_bus = EventBus()
        self.event_bus.set_logger(self.get_logger())
        
        # Initialize all subsystems with dependency injection
        self._initialize_subsystems()
        
        # Set up ROS 2 interfaces after subsystems are ready
        self._setup_ros_interfaces()
        
        # Subscribe to relevant events for coordination
        self._subscribe_to_events()
        
        # Legacy attributes for test compatibility
        self.pipelines = {}  # Deprecated - now handled by PipelineEngine
        self.current_stack = None  # Deprecated - now handled by StackManager
        self.next_stack = None  # Deprecated - now handled by StackManager
        self.method = None  # Deprecated - now extracted from events
        self.stack_parser = create_stack_parser(self.get_logger())  # For test compatibility
        
        self.get_logger().info("Refactored MutoComposer initialized successfully")
    
    def _initialize_subsystems(self):
        """Initialize all subsystems in correct dependency order."""
        try:
            # Initialize core subsystems
            self.message_handler = MessageHandler(
                node=self,
                event_bus=self.event_bus
            )
            
            self.digital_twin = DigitalTwinIntegration(
                node=self,
                event_bus=self.event_bus,
                logger=self.get_logger()
            )
            
            self.stack_manager = StackManager(
                event_bus=self.event_bus,
                logger=self.get_logger()
            )
            
            self.orchestration_manager = OrchestrationManager(
                event_bus=self.event_bus,
                logger=self.get_logger()
            )
            
            self.pipeline_engine = PipelineEngine(
                event_bus=self.event_bus,
                logger=self.get_logger()
            )
            
            self.get_logger().info("All subsystems initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize subsystems: {e}")
            raise
    
    def _setup_ros_interfaces(self):
        """Set up ROS 2 publishers and subscribers."""
        try:
            # Note: MutoAction subscription is now handled by MessageHandler subsystem
            # to avoid duplicate processing
            
            self.get_logger().info("ROS 2 interfaces set up successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to set up ROS interfaces: {e}")
            raise
    
    def _subscribe_to_events(self):
        """Subscribe to coordination events from subsystems."""
        try:
            # Subscribe to events that require high-level coordination
            self.event_bus.subscribe(
                EventType.PIPELINE_COMPLETED,
                self._handle_pipeline_completed
            )
            
            self.event_bus.subscribe(
                EventType.PIPELINE_FAILED,
                self._handle_pipeline_failed
            )
            
            self.get_logger().debug("Event subscriptions set up successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to set up event subscriptions: {e}")
            raise
    
    def on_stack_callback(self, stack_msg: MutoAction):
        """
        Main entry point for handling incoming MutoAction messages.
        Delegates to subsystems via event publishing.
        """
        try:
            self.get_logger().info(f"Received MutoAction: {stack_msg.method}")
            
            # Parse payload
            payload = json.loads(stack_msg.payload)
            
            # Determine stack name (extract from payload or use default)
            stack_name = self._extract_stack_name(payload)
            
            # Create and publish stack request event
            stack_request = StackRequestEvent(
                event_type=EventType.STACK_REQUEST,
                source_component="muto_composer",
                stack_name=stack_name,
                action=stack_msg.method,
                stack_payload=payload
            )
            
            # Publish to event bus for subsystem processing
            self.event_bus.publish_sync(stack_request)
            
            self.get_logger().info(f"Stack request published for processing: {stack_name}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in payload: {e}")
        except Exception as e:
            self.get_logger().error(f"Error handling stack callback: {e}")
    
    def _extract_stack_name(self, payload: Dict[str, Any]) -> str:
        """Extract stack name from payload or generate default."""
        try:
            # Check for value.stackId pattern
            if "value" in payload and isinstance(payload["value"], dict):
                stack_id = payload["value"].get("stackId", "")
                if stack_id:
                    return stack_id
            
            # Check for direct stackId
            stack_id = payload.get("stackId", "")
            if stack_id:
                return stack_id
            
            # Check for metadata name
            if "metadata" in payload:
                name = payload["metadata"].get("name", "")
                if name:
                    return name
            
            # Default naming
            return f"{self.twin_namespace}:{self.name}"
            
        except Exception as e:
            self.get_logger().warning(f"Error extracting stack name: {e}")
            return f"{self.twin_namespace}:{self.name}"
    
    def _handle_pipeline_completed(self, event):
        """Handle pipeline completion for high-level coordination."""
        try:
            self.get_logger().info(f"Pipeline completed: {event.pipeline_name}")
            
            # Log completion details instead of publishing deprecated state
            if hasattr(event, 'final_result') and event.final_result:
                self.get_logger().info(f"Pipeline result keys: {list(event.final_result.keys())}")
            
        except Exception as e:
            self.get_logger().error(f"Error handling pipeline completion: {e}")
    
    def _handle_pipeline_failed(self, event):
        """Handle pipeline failure for error recovery."""
        try:
            self.get_logger().error(f"Pipeline failed: {event.pipeline_name} - {event.error_details}")
            
            # Could implement retry logic or error reporting here
            
        except Exception as e:
            self.get_logger().error(f"Error handling pipeline failure: {e}")
    
    # Legacy interface methods for backward compatibility
    def pipeline_execute(self, pipeline_name: str, additional_context: Optional[Dict] = None, 
                        stack_manifest: Optional[Dict] = None):
        """Legacy interface: Execute a pipeline directly."""
        try:
            self.get_logger().info(f"Legacy pipeline execution request: {pipeline_name}")
            self.pipeline_engine.execute_pipeline(pipeline_name, additional_context, stack_manifest)
        except Exception as e:
            self.get_logger().error(f"Error in legacy pipeline execution: {e}")
    
    # Deprecated methods for test compatibility - marked for removal
    def bootstrap(self):
        """DEPRECATED: Bootstrap method for test compatibility."""
        self.get_logger().warning("bootstrap() method is deprecated")
    
    def activate(self, future):
        """DEPRECATED: Activate method for test compatibility."""
        self.get_logger().warning("activate() method is deprecated")
    
    def set_stack_done_callback(self, future):
        """DEPRECATED: Set stack done callback for test compatibility."""
        self.get_logger().warning("set_stack_done_callback() method is deprecated")
    
    def get_stack_done_callback(self, future):
        """DEPRECATED: Get stack done callback for test compatibility."""
        self.get_logger().warning("get_stack_done_callback() method is deprecated")
    
    def determine_execution_path(self):
        """DEPRECATED: Execution path determination moved to OrchestrationManager."""
        self.get_logger().warning("determine_execution_path() method is deprecated - now handled by OrchestrationManager")
    
    def resolve_expression(self, value: str = "") -> str:
        """DEPRECATED: Expression resolution moved to StackProcessor."""
        self.get_logger().warning("resolve_expression() method is deprecated - now handled by StackProcessor")
        return value  # Return unchanged for compatibility
    
    def merge(self, current_stack: dict, next_stack: dict) -> dict:
        """DEPRECATED: Stack merging moved to StackProcessor."""
        self.get_logger().warning("merge() method is deprecated - now handled by StackProcessor")
        return next_stack  # Return next_stack for basic compatibility
    
    def load_pipeline_config(self, file_path: str):
        """DEPRECATED: Pipeline configuration loading moved to PipelineEngine."""
        self.get_logger().warning("load_pipeline_config() method is deprecated - now handled by PipelineEngine")
        return {"pipelines": []}
    
    def init_pipelines(self, pipeline_config):
        """DEPRECATED: Pipeline initialization moved to PipelineEngine."""
        self.get_logger().warning("init_pipelines() method is deprecated - now handled by PipelineEngine")
    
    # Publisher methods for test compatibility
    def publish_current_stack(self, stack: str):
        """DEPRECATED: Current stack publishing removed per guidelines."""
        self.get_logger().warning("publish_current_stack() method is deprecated")
    
    def publish_next_stack(self, stack: str):
        """DEPRECATED: Next stack publishing removed per guidelines."""
        self.get_logger().warning("publish_next_stack() method is deprecated")
    
    def publish_raw_stack(self, stack: str):
        """DEPRECATED: Raw stack publishing removed per guidelines."""
        self.get_logger().warning("publish_raw_stack() method is deprecated")
    
    def parse_payload(self, payload):
        """DEPRECATED: Payload parsing moved to StackProcessor.""" 
        self.get_logger().warning("parse_payload() method is deprecated - now handled by StackProcessor")
        return payload
    
    def extract_stack_from_solution(self, solution):
        """DEPRECATED: Stack extraction functionality moved to StackProcessor."""
        self.get_logger().warning("extract_stack_from_solution() method is deprecated")
        return solution


def main(args=None):
    """Main entry point for the Muto Composer node."""
    try:
        rclpy.init(args=args)
        composer = MutoComposer()
        rclpy.spin(composer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        try:
            composer.destroy_node()
        except:
            pass
        
        if rclpy.ok():
            rclpy.shutdown()
