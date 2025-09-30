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
Event system for the refactored Muto Composer.
Provides event-driven communication between subsystems.
"""

import uuid
import asyncio
from datetime import datetime
from enum import Enum
from typing import Dict, Any, Optional, List, Callable
from concurrent.futures import ThreadPoolExecutor

import uuid
from datetime import datetime
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List, Callable
import asyncio
from concurrent.futures import ThreadPoolExecutor


class EventType(Enum):
    """Enumeration of all event types in the composer system."""
    
    # Stack Events
    STACK_REQUEST = "stack.request"
    STACK_ANALYZED = "stack.analyzed"
    STACK_PROCESSED = "stack.processed"
    STACK_MERGED = "stack.merged"
    STACK_VALIDATED = "stack.validated"
    STACK_TRANSFORMED = "stack.transformed"
    
    # Orchestration Events
    ORCHESTRATION_STARTED = "orchestration.started"
    ORCHESTRATION_COMPLETED = "orchestration.completed"
    ORCHESTRATION_FAILED = "orchestration.failed"
    
    # Pipeline Events
    PIPELINE_REQUESTED = "pipeline.requested"
    PIPELINE_START = "pipeline.start"
    PIPELINE_STARTED = "pipeline.started"
    PIPELINE_STEP_STARTED = "pipeline.step.started"
    PIPELINE_STEP_COMPLETED = "pipeline.step.completed"
    PIPELINE_STEP_FAILED = "pipeline.step.failed"
    PIPELINE_COMPLETE = "pipeline.complete"
    PIPELINE_COMPLETED = "pipeline.completed"
    PIPELINE_ERROR = "pipeline.error"
    PIPELINE_FAILED = "pipeline.failed"
    PIPELINE_COMPENSATION_STARTED = "pipeline.compensation.started"
    
    # Plugin Operation Events
    COMPOSE_REQUESTED = "compose.requested"
    COMPOSE_COMPLETED = "compose.completed"
    PROVISION_REQUESTED = "provision.requested"
    PROVISION_COMPLETED = "provision.completed"
    LAUNCH_REQUESTED = "launch.requested"
    LAUNCH_COMPLETED = "launch.completed"
    
    # System Events
    TWIN_UPDATE = "twin.update"
    TWIN_SYNC_REQUESTED = "twin.sync.requested"
    TWIN_SYNC_COMPLETED = "twin.sync.completed"
    CONFIGURATION_CHANGED = "config.changed"


class BaseComposeEvent:
    """Base class for all composer events."""
    
    def __init__(self, event_type: EventType, source_component: str, 
                 event_id: Optional[str] = None, timestamp: Optional[datetime] = None,
                 correlation_id: Optional[str] = None, metadata: Optional[Dict[str, Any]] = None,
                 # Common attributes used across multiple event types
                 stack_payload: Optional[Dict[str, Any]] = None,
                 stack_name: Optional[str] = None,
                 action: Optional[str] = None,
                 pipeline_name: Optional[str] = None,
                 execution_context: Optional[Dict[str, Any]] = None,
                 orchestration_id: Optional[str] = None):
        self.event_type = event_type
        self.source_component = source_component
        self.event_id = event_id or str(uuid.uuid4())
        self.timestamp = timestamp or datetime.now()
        self.correlation_id = correlation_id
        self.metadata = metadata or {}
        
        # Common attributes
        self.stack_payload = stack_payload or {}
        self.stack_name = stack_name
        self.action = action
        self.pipeline_name = pipeline_name
        self.execution_context = execution_context or {}
        self.orchestration_id = orchestration_id


class StackRequestEvent(BaseComposeEvent):
    """Event triggered when a stack operation is requested."""
    
    def __init__(self, event_type: EventType, source_component: str, stack_name: str, action: str,
                 stack_payload: Optional[Dict[str, Any]] = None, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            stack_name=stack_name,
            action=action,
            stack_payload=stack_payload,
            **kwargs
        )


class StackAnalyzedEvent(BaseComposeEvent):
    """Event triggered when stack analysis is complete."""
    
    def __init__(self, event_type: EventType, source_component: str, stack_name: str, action: str,
                 analysis_result: Optional[Dict[str, Any]] = None,
                 processing_requirements: Optional[Dict[str, Any]] = None,
                 stack_payload: Optional[Dict[str, Any]] = None, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            stack_name=stack_name,
            action=action,
            stack_payload=stack_payload,
            **kwargs
        )
        self.analysis_result = analysis_result or {}
        self.processing_requirements = processing_requirements or {}
        # Keep manifest_data for backwards compatibility, but map to stack_payload
        self.manifest_data = {"stack_payload": self.stack_payload}


class StackMergedEvent(BaseComposeEvent):
    """Event triggered when stacks are merged."""
    
    def __init__(self, event_type: EventType, source_component: str,
                 current_stack: Optional[Dict[str, Any]] = None,
                 next_stack: Optional[Dict[str, Any]] = None,
                 stack_payload: Optional[Dict[str, Any]] = None,
                 merge_strategy: str = "intelligent_merge",
                 conflicts_resolved: Optional[Dict[str, Any]] = None, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            stack_payload=stack_payload,
            **kwargs
        )
        self.current_stack = current_stack or {}
        self.next_stack = next_stack or {}
        # Keep merged_stack for backwards compatibility, but map to stack_payload
        self.merged_stack = self.stack_payload
        self.merge_strategy = merge_strategy
        self.conflicts_resolved = conflicts_resolved


class StackTransformedEvent(BaseComposeEvent):
    """Event triggered when stack transformation is complete."""
    
    def __init__(self, event_type: EventType, source_component: str,
                 original_stack: Optional[Dict[str, Any]] = None,
                 stack_payload: Optional[Dict[str, Any]] = None,
                 expressions_resolved: Optional[Dict[str, str]] = None,
                 transformation_type: str = "expression_resolution", **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            stack_payload=stack_payload,
            **kwargs
        )
        self.original_stack = original_stack or {}
        # Keep transformed_stack for backwards compatibility, but map to stack_payload
        self.transformed_stack = self.stack_payload
        self.expressions_resolved = expressions_resolved or {}
        self.transformation_type = transformation_type


class OrchestrationStartedEvent(BaseComposeEvent):
    """Event triggered when orchestration process begins."""
    
    def __init__(self, event_type: EventType, source_component: str, action: str,
                 execution_plan: Optional[Dict[str, Any]] = None,
                 context_variables: Optional[Dict[str, Any]] = None,
                 stack_payload: Optional[Dict[str, Any]] = None,
                 orchestration_id: Optional[str] = None, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            action=action,
            stack_payload=stack_payload,
            orchestration_id=orchestration_id or str(uuid.uuid4()),
            **kwargs
        )
        self.execution_plan = execution_plan or {}
        self.context_variables = context_variables or {}


class OrchestrationCompletedEvent(BaseComposeEvent):
    """Event triggered when orchestration completes successfully."""
    
    def __init__(self, event_type: EventType, source_component: str, orchestration_id: str,
                 final_stack_state: Optional[Dict[str, Any]] = None,
                 execution_summary: Optional[Dict[str, Any]] = None,
                 duration: float = 0.0, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            orchestration_id=orchestration_id,
            **kwargs
        )
        self.final_stack_state = final_stack_state or {}
        self.execution_summary = execution_summary or {}
        self.duration = duration


class PipelineRequestedEvent(BaseComposeEvent):
    """Event triggered when a pipeline execution is requested."""
    
    def __init__(self, event_type: EventType, source_component: str, pipeline_name: str,
                 execution_context: Optional[Dict[str, Any]] = None,
                 stack_payload: Optional[Dict[str, Any]] = None, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            pipeline_name=pipeline_name,
            execution_context=execution_context,
            stack_payload=stack_payload,
            **kwargs
        )
        # Keep stack_manifest for backwards compatibility, but map to stack_payload
        self.stack_manifest = self.stack_payload


class PipelineStartedEvent(BaseComposeEvent):
    """Event triggered when a pipeline starts execution."""
    
    def __init__(self, event_type: EventType, source_component: str, pipeline_name: str, execution_id: str,
                 steps_planned: Optional[List[str]] = None, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            pipeline_name=pipeline_name,
            **kwargs
        )
        self.execution_id = execution_id
        self.steps_planned = steps_planned or []


class PipelineCompletedEvent(BaseComposeEvent):
    """Event triggered when a pipeline completes successfully."""
    
    def __init__(self, event_type: EventType, source_component: str, pipeline_name: str, execution_id: str,
                 final_result: Optional[Dict[str, Any]] = None,
                 steps_executed: Optional[List[str]] = None,
                 total_duration: float = 0.0, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            pipeline_name=pipeline_name,
            **kwargs
        )
        self.execution_id = execution_id
        self.final_result = final_result or {}
        self.steps_executed = steps_executed or []
        self.total_duration = total_duration


class PipelineFailedEvent(BaseComposeEvent):
    """Event triggered when a pipeline fails."""
    
    def __init__(self, event_type: EventType, source_component: str, pipeline_name: str, execution_id: str,
                 failure_step: str, error_details: Optional[Dict[str, Any]] = None,
                 compensation_executed: bool = False, **kwargs):
        super().__init__(
            event_type=event_type, 
            source_component=source_component, 
            pipeline_name=pipeline_name,
            **kwargs
        )
        self.execution_id = execution_id
        self.failure_step = failure_step
        self.error_details = error_details or {}
        self.compensation_executed = compensation_executed


class StackProcessedEvent(BaseComposeEvent):
    """Event triggered when stack processing is complete."""
    
    def __init__(self, event_type: EventType = None, source_component: str = "stack_processor", 
                 stack_name: str = "", action: str = "", stack_payload: Optional[Dict[str, Any]] = None,
                 execution_requirements: Optional[Dict[str, Any]] = None,
                 original_payload: Optional[Dict[str, Any]] = None,
                 processing_applied: Optional[list] = None, **kwargs):
        super().__init__(
            event_type=event_type or EventType.STACK_PROCESSED, 
            source_component=source_component, 
            stack_name=stack_name,
            action=action,
            stack_payload=stack_payload,
            **kwargs
        )
        # Keep merged_stack for backwards compatibility, but map to stack_payload
        self.merged_stack = self.stack_payload
        self.execution_requirements = execution_requirements or {}
        self.original_payload = original_payload or {}
        self.processing_applied = processing_applied or []


class TwinUpdateEvent(BaseComposeEvent):
    """Event triggered when a digital twin update is requested."""
    
    def __init__(self, event_type: EventType = None, source_component: str = "twin_integration",
                 twin_id: str = "", update_type: str = "", 
                 data: Optional[Dict[str, Any]] = None, **kwargs):
        super().__init__(
            event_type=event_type or EventType.TWIN_UPDATE, 
            source_component=source_component, 
            **kwargs
        )
        self.twin_id = twin_id
        self.update_type = update_type
        self.data = data or {}


class PipelineEvents:
    """Factory class for creating pipeline-related events."""
    
    @staticmethod
    def create_start_event(pipeline_name: str, context: Optional[Dict[str, Any]] = None):
        """Create a pipeline start event."""
        return PipelineStartedEvent(
            event_type=EventType.PIPELINE_START,
            source_component="pipeline_engine",
            pipeline_name=pipeline_name,
            execution_id=str(uuid.uuid4()),
            metadata=context or {}
        )
    
    @staticmethod
    def create_completion_event(pipeline_name: str, success: bool = True, 
                              result: Optional[Dict[str, Any]] = None):
        """Create a pipeline completion event."""
        return PipelineCompletedEvent(
            event_type=EventType.PIPELINE_COMPLETE,
            source_component="pipeline_engine",
            pipeline_name=pipeline_name,
            execution_id=str(uuid.uuid4()),
            final_result=result or {"success": success}
        )
    
    @staticmethod
    def create_error_event(pipeline_name: str, error: str, 
                          context: Optional[Dict[str, Any]] = None):
        """Create a pipeline error event."""
        return PipelineFailedEvent(
            event_type=EventType.PIPELINE_ERROR,
            source_component="pipeline_engine",
            pipeline_name=pipeline_name,
            execution_id=str(uuid.uuid4()),
            failure_step="unknown",
            error_details={"error": error, "context": context or {}}
        )


class EventBus:
    """Central event bus for composer event handling."""
    
    def __init__(self, max_workers: int = 4):
        self._handlers: Dict[EventType, List[Callable]] = {}
        self._middleware: List[Callable] = []
        self._executor = ThreadPoolExecutor(max_workers=max_workers)
        self._logger = None
    
    def set_logger(self, logger):
        """Set logger for event bus operations."""
        self._logger = logger
    
    def subscribe(self, event_type: EventType, handler: Callable):
        """Subscribe a handler to an event type."""
        if event_type not in self._handlers:
            self._handlers[event_type] = []
        self._handlers[event_type].append(handler)
        
        if self._logger:
            self._logger.debug(f"Subscribed {handler.__name__} to {event_type.value}")
    
    def unsubscribe(self, event_type: EventType, handler: Callable):
        """Unsubscribe a handler from an event type."""
        if event_type in self._handlers:
            self._handlers[event_type].remove(handler)
            
            if self._logger:
                self._logger.debug(f"Unsubscribed {handler.__name__} from {event_type.value}")
    
    def add_middleware(self, middleware: Callable):
        """Add middleware for event processing."""
        self._middleware.append(middleware)
    
    async def publish(self, event: BaseComposeEvent):
        """Publish an event to all subscribers asynchronously."""
        try:
            # Apply middleware
            for middleware in self._middleware:
                event = await middleware(event)
            
            # Get handlers for this event type
            handlers = self._handlers.get(event.event_type, [])
            
            # Execute handlers concurrently
            if handlers:
                tasks = [
                    asyncio.get_event_loop().run_in_executor(
                        self._executor, handler, event
                    )
                    for handler in handlers
                ]
                await asyncio.gather(*tasks, return_exceptions=True)
                
        except Exception as e:
            if self._logger:
                self._logger.error(f"Error publishing event {event.event_type.value}: {e}")
    
    def publish_sync(self, event: BaseComposeEvent):
        """Synchronous event publishing for ROS callbacks."""
        try:
            handlers = self._handlers.get(event.event_type, [])
            
            if self._logger:
                self._logger.debug(f"Publishing {event.event_type.value} to {len(handlers)} handlers")
                
            for handler in handlers:
                try:
                    handler(event)
                except Exception as e:
                    if self._logger:
                        self._logger.error(f"Error in event handler {handler.__name__}: {e}")
                    # Continue with other handlers
                    
        except Exception as e:
            if self._logger:
                self._logger.error(f"Error in synchronous event publishing: {e}")