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
    OrchestrationCompletedEvent, OrchestrationFailedEvent, PipelineRequestedEvent,
    PipelineFailedEvent, RollbackStartedEvent, RollbackCompletedEvent, RollbackFailedEvent
)
from composer.state.persistence import StatePersistence


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

            # Handle kill action specially - it just needs to terminate processes
            if action == "kill" or analysis_result.get("is_kill_action"):
                if self.logger:
                    self.logger.info(f"Kill action detected, executing kill pipeline")
                return ExecutionPath(
                    pipeline_name="kill",
                    context_variables={
                        "should_run_provision": False,
                        "should_run_launch": True,  # LaunchPlugin handles kills
                        "is_kill_action": True,
                        "stack_id": analysis_result.get("stack_id")
                    },
                    requires_merging=False
                )

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
    """Orchestrates complete deployment workflows with rollback support."""

    def __init__(self, event_bus: EventBus, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        self.path_determiner = ExecutionPathDeterminer(logger)
        self.state_persistence = StatePersistence(logger=logger)

        # Subscribe to events
        self.event_bus.subscribe(EventType.STACK_ANALYZED, self.handle_stack_analyzed)
        self.event_bus.subscribe(EventType.STACK_MERGED, self.handle_stack_merged)
        self.event_bus.subscribe(EventType.PIPELINE_FAILED, self.handle_pipeline_failed)

        # Keep track of active orchestrations
        self.active_orchestrations: Dict[str, Dict[str, Any]] = {}
        # Track if we're currently in a rollback to prevent rollback loops
        self._rollback_in_progress: bool = False

        if self.logger:
            self.logger.info("DeploymentOrchestrator initialized with rollback support")
    
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

                # Reset rollback flag if this was a rollback
                if self._rollback_in_progress:
                    self._rollback_in_progress = False
                    if self.logger:
                        self.logger.info("Rollback completed successfully")

                if self.logger:
                    self.logger.info(f"Completed orchestration {orchestration_id}")

        except Exception as e:
            if self.logger:
                self.logger.error(f"Error completing orchestration: {e}")

    def handle_pipeline_failed(self, event: PipelineFailedEvent):
        """Handle pipeline failure and trigger rollback if possible."""
        try:
            # Don't trigger rollback if we're already in a rollback
            if self._rollback_in_progress:
                if self.logger:
                    self.logger.error(
                        f"Rollback failed: {event.pipeline_name} - {event.error_details}"
                    )
                # Publish rollback failed event
                rollback_failed = RollbackFailedEvent(
                    event_type=EventType.ROLLBACK_FAILED,
                    source_component="deployment_orchestrator",
                    orchestration_id=event.execution_id,
                    error_details=str(event.error_details),
                    original_failure="Rollback pipeline failed"
                )
                self.event_bus.publish_sync(rollback_failed)
                self._rollback_in_progress = False
                return

            if self.logger:
                self.logger.error(
                    f"Pipeline failed: {event.pipeline_name} at step {event.failure_step}"
                )

            # Find the orchestration context and get stack name
            stack_name = self._get_stack_name_from_context(event)
            if not stack_name:
                if self.logger:
                    self.logger.warning("Cannot determine stack name for rollback")
                return

            # Check if rollback is possible
            if self.state_persistence.can_rollback(stack_name):
                previous_stack = self.state_persistence.get_previous_stack(stack_name)
                if previous_stack:
                    self.trigger_rollback(stack_name, previous_stack, str(event.error_details))
                else:
                    if self.logger:
                        self.logger.warning(f"No previous stack available for rollback: {stack_name}")
            else:
                if self.logger:
                    self.logger.info(f"Rollback not possible for {stack_name} - no previous version")

                # Publish orchestration failed event without rollback
                failed_event = OrchestrationFailedEvent(
                    event_type=EventType.ORCHESTRATION_FAILED,
                    source_component="deployment_orchestrator",
                    orchestration_id=event.execution_id,
                    error_details=str(event.error_details),
                    failed_step=event.failure_step,
                    can_rollback=False
                )
                self.event_bus.publish_sync(failed_event)

        except Exception as e:
            if self.logger:
                self.logger.error(f"Error handling pipeline failure: {e}")

    def _get_stack_name_from_context(self, event: PipelineFailedEvent) -> Optional[str]:
        """Extract stack name from pipeline failure event context."""
        # Try to find from active orchestrations
        for orch_id, context in self.active_orchestrations.items():
            if hasattr(context.get("event"), "stack_name"):
                return context["event"].stack_name
            # Try to get from stack_payload
            stack_payload = context.get("event", {})
            if hasattr(stack_payload, "stack_payload"):
                metadata = stack_payload.stack_payload.get("metadata", {})
                name = metadata.get("name")
                if name:
                    return name
        return None

    def trigger_rollback(self, stack_name: str, previous_stack: Dict[str, Any], failure_reason: str):
        """Trigger rollback to previous stack version."""
        try:
            if self._rollback_in_progress:
                if self.logger:
                    self.logger.warning("Rollback already in progress, skipping")
                return

            self._rollback_in_progress = True

            if self.logger:
                self.logger.info(f"Triggering rollback for {stack_name} due to: {failure_reason}")

            # Get the current (failed) stack for reference
            state = self.state_persistence.load_state(stack_name)
            failed_stack = state.current_stack if state else {}

            # Publish rollback started event
            rollback_event = RollbackStartedEvent(
                event_type=EventType.ROLLBACK_STARTED,
                source_component="deployment_orchestrator",
                previous_stack=previous_stack,
                failed_stack=failed_stack,
                failure_reason=failure_reason
            )
            self.event_bus.publish_sync(rollback_event)

            # Create a new orchestration for the rollback using the previous stack
            orchestration_id = str(uuid.uuid4())

            # Create execution path for rollback
            execution_path = ExecutionPath(
                pipeline_name="rollback",
                context_variables={
                    "should_run_provision": True,  # May need to provision previous version
                    "should_run_launch": True,
                    "is_rollback": True
                },
                requires_merging=False
            )

            # Store rollback orchestration context
            self.active_orchestrations[orchestration_id] = {
                "execution_path": execution_path,
                "status": "rollback_started",
                "previous_stack": previous_stack,
                "failed_stack": failed_stack
            }

            # Trigger orchestration with previous stack
            orchestration_event = OrchestrationStartedEvent(
                event_type=EventType.ORCHESTRATION_STARTED,
                source_component="deployment_orchestrator",
                orchestration_id=orchestration_id,
                action="rollback",
                execution_plan=execution_path.to_dict(),
                context_variables=execution_path.context_variables,
                stack_payload=previous_stack,
                metadata={"is_rollback": True, "failure_reason": failure_reason}
            )

            self.event_bus.publish_sync(orchestration_event)

            if self.logger:
                self.logger.info(f"Rollback orchestration {orchestration_id} started")

        except Exception as e:
            self._rollback_in_progress = False
            if self.logger:
                self.logger.error(f"Error triggering rollback: {e}")


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