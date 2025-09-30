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
Stack management subsystem for the refactored Muto Composer.
Handles stack states, analysis, and transformations.
"""

import os
import re
import json
from typing import Dict, Any, Optional
from enum import Enum
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
from composer.model.stack import Stack
from composer.utils.stack_parser import create_stack_parser
from composer.events import (
    EventBus, EventType, StackRequestEvent, StackAnalyzedEvent,
    StackMergedEvent, StackTransformedEvent, StackProcessedEvent
)


class StackType(Enum):
    """Enumeration of stack types."""
    ARCHIVE = "stack/archive"
    JSON = "stack/json" 
    RAW = "stack/raw"
    LEGACY = "stack/legacy"
    UNKNOWN = "stack/unknown"


@dataclass
class ExecutionRequirements:
    """Stack execution requirements."""
    requires_provision: bool = False
    requires_launch: bool = False
    has_nodes: bool = False
    has_composables: bool = False
    has_launch_description: bool = False
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "requires_provision": self.requires_provision,
            "requires_launch": self.requires_launch,
            "has_nodes": self.has_nodes,
            "has_composables": self.has_composables,
            "has_launch_description": self.has_launch_description
        }


@dataclass
class StackTransition:
    """Represents a transition between stack states."""
    current: Optional[Dict[str, Any]] = None
    next: Optional[Dict[str, Any]] = None
    transition_type: str = "deploy"


class StackStateManager:
    """Manages current and next stack states."""
    
    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        self.current_stack: Optional[Dict] = None
        self.next_stack: Optional[Dict] = None
        
        # Subscribe to events
        self.event_bus.subscribe(EventType.STACK_MERGED, self.handle_stack_merged)
        self.event_bus.subscribe(EventType.ORCHESTRATION_COMPLETED, self.handle_orchestration_completed)
        
        if self.logger:
            self.logger.info("StackStateManager initialized")
    
    def set_current_stack(self, stack: Dict) -> None:
        """Update current stack state."""
        self.current_stack = stack
        if self.logger:
            self.logger.debug("Current stack updated")
    
    def set_next_stack(self, stack: Dict) -> None:
        """Set stack for next deployment."""
        self.next_stack = stack
        if self.logger:
            self.logger.debug("Next stack set")
    
    def get_current_stack(self) -> Optional[Dict]:
        """Get current stack."""
        return self.current_stack
    
    def get_next_stack(self) -> Optional[Dict]:
        """Get next stack."""
        return self.next_stack
    
    def get_stack_transition(self) -> StackTransition:
        """Calculate transition from current to next."""
        return StackTransition(
            current=self.current_stack,
            next=self.next_stack,
            transition_type=self._determine_transition_type()
        )
    
    def _determine_transition_type(self) -> str:
        """Determine the type of transition."""
        if not self.current_stack:
            return "initial_deploy"
        elif not self.next_stack:
            return "shutdown"
        else:
            return "update"
    
    def handle_stack_merged(self, event: StackMergedEvent):
        """Handle stack merged event."""
        self.set_current_stack(event.merged_stack)
        if self.logger:
            self.logger.info("Updated current stack from merge event")
    
    def handle_orchestration_completed(self, event):
        """Handle orchestration completion."""
        if hasattr(event, 'final_stack_state') and event.final_stack_state:
            self.set_current_stack(event.final_stack_state)
            if self.logger:
                self.logger.info("Updated current stack from orchestration completion")


class StackAnalyzer:
    """Analyzes stack characteristics and determines execution requirements."""
    
    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        
        # Subscribe to stack request events
        self.event_bus.subscribe(EventType.STACK_REQUEST, self.handle_stack_request)
        
        if self.logger:
            self.logger.info("StackAnalyzer initialized")
    
    def analyze_stack_type(self, stack: Dict) -> StackType:
        """Determine if stack is archive, JSON, raw, or legacy."""
        metadata = stack.get("metadata", {})
        content_type = metadata.get("content_type", "")
        
        # Check for new prefixed format first
        if content_type == StackType.ARCHIVE.value:
            return StackType.ARCHIVE
        elif content_type == StackType.JSON.value:
            return StackType.JSON
        elif content_type == StackType.RAW.value:
            return StackType.RAW
        elif content_type == StackType.LEGACY.value:
            return StackType.LEGACY
        # Check for legacy format without prefix for backward compatibility
        elif content_type == StackType.ARCHIVE.value.replace("stack/", ""):
            return StackType.ARCHIVE
        elif content_type == StackType.JSON.value.replace("stack/", ""):
            return StackType.JSON
        elif content_type == StackType.RAW.value.replace("stack/", ""):
            return StackType.RAW
        elif content_type == StackType.LEGACY.value.replace("stack/", ""):
            return StackType.LEGACY
        # Fallback analysis based on stack structure
        elif stack.get("node") or stack.get("composable"):
            return StackType.RAW
        elif stack.get("launch_description_source") or (stack.get("on_start") and stack.get("on_kill")):
            return StackType.LEGACY
        else:
            return StackType.UNKNOWN
    
    def determine_execution_requirements(self, stack: Dict) -> ExecutionRequirements:
        """Calculate provisioning and launch requirements."""
        stack_type = self.analyze_stack_type(stack)
        
        return ExecutionRequirements(
            requires_provision=stack_type == StackType.ARCHIVE,
            requires_launch=stack_type in [StackType.ARCHIVE, StackType.JSON, StackType.RAW],
            has_nodes=bool(stack.get("node")),
            has_composables=bool(stack.get("composable")),
            has_launch_description=bool(stack.get("launch_description_source"))
        )
    
    def handle_stack_request(self, event: StackRequestEvent):
        """Handle stack request by analyzing the payload."""
        try:
            stack_payload = event.stack_payload or {}
            stack_type = self.analyze_stack_type(stack_payload)
            requirements = self.determine_execution_requirements(stack_payload)
            
            analyzed_event = StackAnalyzedEvent(
                event_type=EventType.STACK_ANALYZED,
                source_component="stack_analyzer",
                stack_name=event.stack_name,
                action=event.action,
                analysis_result={
                    "stack_type": stack_type.value,
                    "content_type": stack_payload.get("metadata", {}).get("content_type"),
                    "requires_provision": requirements.requires_provision,
                    "requires_launch": requirements.requires_launch,
                    "has_nodes": requirements.has_nodes,
                    "has_composables": requirements.has_composables,
                    "has_launch_description": requirements.has_launch_description
                },
                processing_requirements=requirements.to_dict(),
                stack_payload=stack_payload,  # Use direct field instead of nested structure
                correlation_id=event.correlation_id
            )
            
            if self.logger:
                self.logger.info(f"Analyzed stack as {stack_type.value}, requires_provision={requirements.requires_provision}")
            
            self.event_bus.publish_sync(analyzed_event)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error analyzing stack: {e}")


class StackProcessor:
    """Handles stack transformations and merging."""
    
    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        self.stack_parser = create_stack_parser(logger)
        
        # Subscribe to events that require processing
        self.event_bus.subscribe(EventType.STACK_ANALYZED, self.handle_stack_analyzed)
        
        if self.logger:
            self.logger.info("StackProcessor initialized")
    
    def handle_stack_analyzed(self, event: StackAnalyzedEvent):
        """Handle stack analyzed event and perform required processing."""
        try:
            processing_requirements = event.processing_requirements
            stack_payload = event.manifest_data.get("stack_payload", {})
            processed_payload = stack_payload
            processing_applied = False
            
            # Check if merging is required
            if processing_requirements.get("merge_manifests", False):
                # For now, we'll simulate merging with current stack
                # In a full implementation, this would get current stack from state manager
                current_stack = {}  # Would be retrieved from state manager
                processed_payload = self.merge_stacks(current_stack, processed_payload)
                processing_applied = True
                
                if self.logger:
                    self.logger.info("Stack merging completed as required by analysis")
            
            # Check if expression resolution is required
            if processing_requirements.get("resolve_expressions", False):
                resolved_json = self.resolve_expressions(json.dumps(processed_payload))
                processed_payload = json.loads(resolved_json)
                processing_applied = True
                
                if self.logger:
                    self.logger.info("Expression resolution completed as required by analysis")
            
            # If any processing was applied, emit a processed event with updated payload
            if processing_applied:
                processed_event = StackProcessedEvent(
                    event_type=EventType.STACK_PROCESSED,
                    source_component="stack_processor",
                    correlation_id=event.correlation_id,
                    stack_name=event.stack_name,
                    action=event.action,
                    stack_payload=processed_payload,
                    original_payload=stack_payload,
                    processing_applied=list(processing_requirements.keys())
                )
                self.event_bus.publish_async(processed_event)
                
                if self.logger:
                    self.logger.info(f"Published processed stack event with applied processing: {processing_requirements}")
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error processing analyzed stack: {e}")
    
    def merge_stacks(self, current: Dict, next: Dict) -> Dict:
        """Merge current and next stacks intelligently."""
        try:
            if not current:
                current = {}
            
            stack_1 = Stack(manifest=current)
            stack_2 = Stack(manifest=next)
            merged = stack_1.merge(stack_2)
            
            # Publish merge event
            merge_event = StackMergedEvent(
                event_type=EventType.STACK_MERGED,
                source_component="stack_processor",
                current_stack=current,
                next_stack=next,
                stack_payload=merged.manifest,
                merge_strategy="intelligent_merge"
            )
            self.event_bus.publish_sync(merge_event)
            
            if self.logger:
                self.logger.info("Successfully merged stacks")
            
            return merged.manifest
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error merging stacks: {e}")
            return next  # Fallback to next stack
    
    def resolve_expressions(self, stack_json: str, current_stack: Optional[Dict] = None) -> str:
        """Resolve dynamic expressions in stack definitions."""
        try:
            expressions = re.findall(r"\$\(([\s0-9a-zA-Z_-]+)\)", stack_json)
            result = stack_json
            resolved_expressions = {}
            
            for expression in expressions:
                parts = expression.split()
                if len(parts) != 2:
                    if self.logger:
                        self.logger.warning(f"Invalid expression format: {expression}")
                    continue
                    
                expr, var = parts
                resolved_value = ""
                
                try:
                    if expr == "find":
                        resolved_value = get_package_share_directory(var)
                    elif expr == "env":
                        resolved_value = os.getenv(var, "")
                    elif expr == "arg":
                        if current_stack:
                            resolved_value = current_stack.get("args", {}).get(var, "")
                        if self.logger:
                            self.logger.info(f"Resolved arg {var}: {resolved_value}")
                    
                    resolved_expressions[expression] = resolved_value
                    result = re.sub(
                        r"\$\(" + re.escape(expression) + r"\)",
                        resolved_value,
                        result,
                        count=1,
                    )
                except Exception as e:
                    if self.logger:
                        self.logger.warning(f"Failed to resolve expression {expression}: {e}")
                    continue
            
            # Publish transformation event if any expressions were resolved
            if resolved_expressions:
                transform_event = StackTransformedEvent(
                    event_type=EventType.STACK_TRANSFORMED,
                    source_component="stack_processor",
                    original_stack=json.loads(stack_json),
                    stack_payload=json.loads(result),
                    expressions_resolved=resolved_expressions,
                    transformation_type="expression_resolution"
                )
                self.event_bus.publish_sync(transform_event)
                
                if self.logger:
                    self.logger.info(f"Resolved {len(resolved_expressions)} expressions")
            
            return result
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error resolving expressions: {e}")
            return stack_json  # Return original on error
    
    def parse_payload(self, payload: Dict) -> Dict:
        """Parse and normalize different payload formats."""
        try:
            parsed = self.stack_parser.parse_payload(payload)
            if parsed and parsed != payload:
                if self.logger:
                    self.logger.info("Parsed stack payload using stack parser utility")
                return parsed
            return payload
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error parsing payload: {e}")
            return payload


class StackManager:
    """Main stack management subsystem coordinator."""
    
    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        
        # Initialize components
        self.state_manager = StackStateManager(event_bus, logger)
        self.analyzer = StackAnalyzer(event_bus, logger)
        self.processor = StackProcessor(event_bus, logger)
        
        if self.logger:
            self.logger.info("StackManager subsystem initialized")
    
    def get_state_manager(self) -> StackStateManager:
        """Get state manager."""
        return self.state_manager
    
    def get_analyzer(self) -> StackAnalyzer:
        """Get analyzer."""
        return self.analyzer
    
    def get_processor(self) -> StackProcessor:
        """Get processor."""
        return self.processor