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
Orchestration management subsystem for the refactored Muto Composer.
Handles high-level deployment workflows and coordination.
"""

import uuid
from typing import Dict, Any, Optional
from composer.subsystems.stack_manager import StackType
from dataclasses import dataclass
from composer.events import (
    EventBus, EventType, StackAnalyzedEvent, OrchestrationStartedEvent,
    OrchestrationCompletedEvent, PipelineRequestedEvent
)


@dataclass
class ExecutionPath:
    """Represents an execution path for stack deployment."""
    pipeline_name: str
    context_variables: Dict[str, Any]
    requires_merging: bool = False
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "pipeline_name": self.pipeline_name,
            "context_variables": self.context_variables,
            "requires_merging": self.requires_merging
        }


class ExecutionPathDeterminer:
    """Determines execution path based on stack analysis."""
    
    def __init__(self, logger=None):
        self.logger = logger
    
    def determine_path(self, 
                      analyzed_event: StackAnalyzedEvent,
                      current_stack: Optional[Dict] = None,
                      next_stack: Optional[Dict] = None) -> ExecutionPath:
        """Determine execution path and context variables."""
        
        try:
            # Extract information from analyzed event
            analysis_result = analyzed_event.analysis_result
            stack_type = analysis_result.get("stack_type", StackType.UNKNOWN.value)
            action = analyzed_event.action
            stack_payload = analyzed_event.stack_payload  # Use direct field instead of nested lookup
            
            # Complex logic extracted from original determine_execution_path method
            is_next_stack_empty = (not stack_payload.get("node", "") and 
                                 not stack_payload.get("composable", ""))
            has_launch_description = bool(stack_payload.get("launch_description_source"))
            has_on_start_and_on_kill = all([
                stack_payload.get("on_start"), 
                stack_payload.get("on_kill")
            ])
            
            # Determine execution requirements based on stack type and characteristics
            if stack_type == StackType.ARCHIVE.value:
                should_run_provision = True
                should_run_launch = True
                requires_merging = False
                if self.logger:
                    self.logger.info("Archive manifest detected; running ProvisionPlugin and LaunchPlugin")
                    
            elif stack_type == StackType.JSON.value:
                should_run_provision = False
                should_run_launch = True
                requires_merging = True
                if self.logger:
                    self.logger.info("JSON manifest detected; running LaunchPlugin")
                    
            elif is_next_stack_empty and (has_launch_description or has_on_start_and_on_kill):
                should_run_provision = False
                should_run_launch = True
                requires_merging = False
                if self.logger:
                    self.logger.info("Legacy stack conditions met to run LaunchPlugin")
                    
            elif not is_next_stack_empty:
                should_run_provision = False
                should_run_launch = True
                requires_merging = True
                if self.logger:
                    self.logger.info("Conditions met to merge stacks and bypass ProvisionPlugin")
                    
            else:
                should_run_provision = False
                should_run_launch = False
                requires_merging = False
                if self.logger:
                    self.logger.info("Conditions not met to run ProvisionPlugin AND LaunchPlugin")
            
            context_variables = {
                "should_run_provision": should_run_provision,
                "should_run_launch": should_run_launch,
            }
            
            return ExecutionPath(
                pipeline_name=action,
                context_variables=context_variables,
                requires_merging=requires_merging
            )
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error determining execution path: {e}")
            
            # Fallback path
            return ExecutionPath(
                pipeline_name=analyzed_event.action,
                context_variables={"should_run_provision": False, "should_run_launch": False},
                requires_merging=False
            )


class DeploymentOrchestrator:
    """Orchestrates complete deployment workflows."""
    
    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        self.path_determiner = ExecutionPathDeterminer(logger)
        
        # Subscribe to events
        self.event_bus.subscribe(EventType.STACK_ANALYZED, self.handle_stack_analyzed)
        self.event_bus.subscribe(EventType.STACK_MERGED, self.handle_stack_merged)
        
        # Keep track of active orchestrations
        self.active_orchestrations: Dict[str, Dict[str, Any]] = {}
        
        if self.logger:
            self.logger.info("DeploymentOrchestrator initialized")
    
    def handle_stack_analyzed(self, event: StackAnalyzedEvent):
        """Handle analyzed stack by determining orchestration path."""
        try:
            execution_path = self.path_determiner.determine_path(event)
            
            orchestration_id = str(uuid.uuid4())
            
            # Store orchestration context
            self.active_orchestrations[orchestration_id] = {
                "event": event,
                "execution_path": execution_path,
                "status": "started"
            }
            
            orchestration_event = OrchestrationStartedEvent(
                event_type=EventType.ORCHESTRATION_STARTED,
                source_component="deployment_orchestrator",
                correlation_id=event.correlation_id,
                orchestration_id=orchestration_id,
                action=event.metadata.get("action", "unknown"),
                execution_plan=execution_path.to_dict(),
                context_variables=execution_path.context_variables,
                stack_payload=event.stack_payload,  # Pass stack_payload directly from analyzed event
                metadata={
                    "requires_merging": execution_path.requires_merging
                }
            )
            
            if self.logger:
                self.logger.info(f"Starting orchestration {orchestration_id} for {event.metadata.get('action')}")
            
            self.event_bus.publish_sync(orchestration_event)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error handling stack analyzed event: {e}")
    
    def handle_stack_merged(self, event):
        """Handle stack merged event - may trigger pipeline execution."""
        # This could be used to continue orchestration after stack merging
        if self.logger:
            self.logger.debug("Stack merged event received in orchestrator")
    
    def complete_orchestration(self, orchestration_id: str, final_stack_state: Dict[str, Any]):
        """Complete an orchestration."""
        try:
            if orchestration_id in self.active_orchestrations:
                orchestration_context = self.active_orchestrations[orchestration_id]
                orchestration_context["status"] = "completed"
                
                completion_event = OrchestrationCompletedEvent(
                    event_type=EventType.ORCHESTRATION_COMPLETED,
                    source_component="deployment_orchestrator",
                    orchestration_id=orchestration_id,
                    final_stack_state=final_stack_state,
                    execution_summary={"status": "success"},
                    duration=0.0  # Would be calculated in real implementation
                )
                
                self.event_bus.publish_sync(completion_event)
                
                # Clean up
                del self.active_orchestrations[orchestration_id]
                
                if self.logger:
                    self.logger.info(f"Completed orchestration {orchestration_id}")
                    
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error completing orchestration: {e}")


class OrchestrationManager:
    """Main orchestration management subsystem coordinator."""
    
    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        
        # Initialize components
        self.orchestrator = DeploymentOrchestrator(event_bus, logger)
        
        if self.logger:
            self.logger.info("OrchestrationManager subsystem initialized")
    
    def get_orchestrator(self) -> DeploymentOrchestrator:
        """Get deployment orchestrator."""
        return self.orchestrator