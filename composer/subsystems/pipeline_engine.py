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
Pipeline engine subsystem for the refactored Muto Composer.
Manages pipeline configurations and execution.
"""

import os
import yaml
import uuid
from typing import Dict, Any, Optional
from ament_index_python.packages import get_package_share_directory
from jsonschema import validate, ValidationError
from composer.workflow.pipeline import Pipeline
from composer.workflow.schemas.pipeline_schema import PIPELINE_SCHEMA
from composer.events import (
    EventBus, EventType, OrchestrationStartedEvent, PipelineRequestedEvent,
    PipelineStartedEvent, PipelineCompletedEvent, PipelineFailedEvent
)


class PipelineManager:
    """Manages pipeline configurations and lifecycle."""
    
    def __init__(self, config_path: Optional[str] = None, logger=None):
        self.logger = logger
        self.pipelines: Dict[str, Pipeline] = {}
        
        # Set default config path if not provided
        if not config_path:
            config_path = os.path.join(
                get_package_share_directory("composer"), "config", "pipeline.yaml"
            )
        
        self.config_path = config_path
        self._load_and_initialize_pipelines()
        
        if self.logger:
            self.logger.info(f"PipelineManager initialized with {len(self.pipelines)} pipelines")
    
    def _load_and_initialize_pipelines(self):
        """Load and initialize all configured pipelines."""
        try:
            config = self.load_pipeline_config(self.config_path)
            self.initialize_pipelines(config)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to initialize pipelines: {e}")
            raise
    
    def load_pipeline_config(self, config_path: str) -> Dict[str, Any]:
        """Load and validate pipeline configuration."""
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            
            validate(instance=config, schema=PIPELINE_SCHEMA)
            
            if self.logger:
                self.logger.info(f"Loaded pipeline configuration from {config_path}")
            
            return config
            
        except FileNotFoundError:
            if self.logger:
                self.logger.error(f"Pipeline configuration file not found: {config_path}")
            raise
        except ValidationError as e:
            if self.logger:
                self.logger.error(f"Invalid pipeline configuration: {e}")
            raise ValueError(f"Invalid pipeline configuration: {e}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error loading pipeline configuration: {e}")
            raise
    
    def initialize_pipelines(self, config: Dict[str, Any]):
        """Initialize all configured pipelines."""
        try:
            loaded_pipelines = {}
            
            for pipeline_item in config.get("pipelines", []):
                name = pipeline_item["name"]
                pipeline_spec = pipeline_item["pipeline"]
                compensation_spec = pipeline_item.get("compensation", None)
                
                pipeline = Pipeline(name, pipeline_spec, compensation_spec)
                loaded_pipelines[name] = pipeline
                
                if self.logger:
                    self.logger.debug(f"Initialized pipeline: {name}")
            
            self.pipelines = loaded_pipelines
            
            if self.logger:
                self.logger.info(f"Successfully initialized {len(loaded_pipelines)} pipelines")
                
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error initializing pipelines: {e}")
            raise
    
    def get_pipeline(self, name: str) -> Optional[Pipeline]:
        """Retrieve pipeline by name."""
        pipeline = self.pipelines.get(name)
        if not pipeline and self.logger:
            self.logger.warning(f"Pipeline '{name}' not found")
        return pipeline
    
    def get_available_pipelines(self) -> Dict[str, Pipeline]:
        """Get all available pipelines."""
        return self.pipelines.copy()
    
    def reload_configuration(self):
        """Reload pipeline configuration from file."""
        try:
            self._load_and_initialize_pipelines()
            if self.logger:
                self.logger.info("Pipeline configuration reloaded successfully")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to reload pipeline configuration: {e}")
            raise


class PipelineExecutor:
    """Executes pipelines with context and error handling."""
    
    def __init__(self, event_bus: EventBus, pipeline_manager: PipelineManager, logger=None):
        self.event_bus = event_bus
        self.pipeline_manager = pipeline_manager
        self.logger = logger
        
        # Subscribe to orchestration events
        self.event_bus.subscribe(EventType.ORCHESTRATION_STARTED, self.handle_orchestration_started)
        
        # Track active executions
        self.active_executions: Dict[str, Dict[str, Any]] = {}
        
        if self.logger:
            self.logger.info("PipelineExecutor initialized")
    
    def handle_orchestration_started(self, event: OrchestrationStartedEvent):
        """Handle orchestration start by executing appropriate pipeline."""
        try:
            pipeline_name = event.execution_plan.get("pipeline_name", event.action)
            context = event.context_variables
            
            # Check if stack merging is required first
            requires_merging = event.metadata.get("requires_merging", False)
            stack_payload = event.stack_payload  # Use direct field instead of metadata
            
            if requires_merging:
                # For now, we'll proceed directly to pipeline execution
                # In a full implementation, this would wait for stack merging to complete
                if self.logger:
                    self.logger.info("Stack merging required, proceeding with pipeline execution")
            
            pipeline_event = PipelineRequestedEvent(
                event_type=EventType.PIPELINE_REQUESTED,
                source_component="pipeline_executor",
                correlation_id=event.correlation_id,
                pipeline_name=pipeline_name,
                execution_context=context,
                stack_payload=stack_payload  # Use consistent naming
            )
            
            if self.logger:
                self.logger.info(f"Requesting pipeline execution: {pipeline_name}")
            
            self.event_bus.publish_sync(pipeline_event)
            self._execute_pipeline_internal(pipeline_event)
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error handling orchestration started: {e}")
    
    def _execute_pipeline_internal(self, event: PipelineRequestedEvent):
        """Internal pipeline execution logic."""
        execution_id = str(uuid.uuid4())
        
        try:
            pipeline = self.pipeline_manager.get_pipeline(event.pipeline_name)
            if not pipeline:
                self._publish_pipeline_failed(event, execution_id, "pipeline_lookup", 
                                            f"No pipeline found: {event.pipeline_name}")
                return
            
            # Store execution context
            self.active_executions[execution_id] = {
                "event": event,
                "pipeline": pipeline,
                "status": "running"
            }
            
            # Publish pipeline started event
            self.event_bus.publish_sync(PipelineStartedEvent(
                event_type=EventType.PIPELINE_STARTED,
                source_component="pipeline_executor",
                correlation_id=event.correlation_id,
                pipeline_name=event.pipeline_name,
                execution_id=execution_id,
                steps_planned=self._extract_step_names(pipeline)
            ))
            
            if self.logger:
                self.logger.info(f"Starting pipeline execution: {event.pipeline_name} [{execution_id}]")
            
            # Execute pipeline
            result = self._execute_pipeline_real(pipeline, event)
            
            # Check if pipeline execution was successful
            if result.get("success", False):
                # Publish completion event
                self.event_bus.publish_sync(PipelineCompletedEvent(
                    event_type=EventType.PIPELINE_COMPLETED,
                    source_component="pipeline_executor",
                    correlation_id=event.correlation_id,
                    pipeline_name=event.pipeline_name,
                    execution_id=execution_id,
                    final_result=result,
                    steps_executed=self._extract_step_names(pipeline),
                    total_duration=0.0  # Would be calculated in real implementation
                ))
                
                if self.logger:
                    self.logger.info(f"Pipeline execution completed: {event.pipeline_name} [{execution_id}]")
            else:
                # Pipeline failed, publish failure event
                self._publish_pipeline_failed(event, execution_id, "execution", 
                                            result.get("error", "Pipeline execution failed"))
                if self.logger:
                    self.logger.error(f"Pipeline execution failed: {event.pipeline_name} [{execution_id}]")
            
            # Clean up
            if execution_id in self.active_executions:
                del self.active_executions[execution_id]
                
        except Exception as e:
            self._publish_pipeline_failed(event, execution_id, "execution", str(e))
            
            # Clean up
            if execution_id in self.active_executions:
                del self.active_executions[execution_id]
    
    def _execute_pipeline_real(self, pipeline: Pipeline, event: PipelineRequestedEvent) -> Dict[str, Any]:
        """Execute pipeline for real."""
        try:
            if self.logger:
                self.logger.info(f"Executing pipeline: {pipeline.name}")
            
            # Execute the actual pipeline
            pipeline.execute_pipeline(
                additional_context=event.execution_context,
                next_manifest=event.stack_manifest
            )
            
            # Return pipeline context as result
            return {
                "success": True,
                "pipeline": pipeline.name,
                "context": pipeline.context,
                "execution_context": event.execution_context
            }
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Pipeline execution failed: {pipeline.name} - {e}")
            
            return {
                "success": False,
                "pipeline": pipeline.name,
                "error": str(e),
                "context": getattr(pipeline, 'context', {}),
                "execution_context": event.execution_context
            }
    
    def _extract_step_names(self, pipeline: Pipeline) -> list:
        """Extract step names from pipeline for reporting."""
        step_names = []
        try:
            for item in pipeline.steps:
                sequence = item.get("sequence", [])
                for step in sequence:
                    step_name = step.get("name", step.get("service", "unknown"))
                    step_names.append(step_name)
        except Exception:
            pass
        return step_names
    
    def _publish_pipeline_failed(self, event: PipelineRequestedEvent, execution_id: str, 
                                failure_step: str, error_message: str):
        """Publish pipeline failure event."""
        self.event_bus.publish_sync(PipelineFailedEvent(
            event_type=EventType.PIPELINE_FAILED,
            source_component="pipeline_executor",
            correlation_id=event.correlation_id,
            pipeline_name=event.pipeline_name,
            execution_id=execution_id,
            failure_step=failure_step,
            error_details={"error": error_message},
            compensation_executed=False
        ))
        
        if self.logger:
            self.logger.error(f"Pipeline execution failed: {event.pipeline_name} [{execution_id}] - {error_message}")


class PipelineEngine:
    """Main pipeline engine subsystem coordinator."""
    
    def __init__(self, event_bus: EventBus, config_path: Optional[str] = None, logger=None):
        self.event_bus = event_bus
        self.logger = logger
        
        # Initialize components
        self.manager = PipelineManager(config_path, logger)
        self.executor = PipelineExecutor(event_bus, self.manager, logger)
        
        if self.logger:
            self.logger.info("PipelineEngine subsystem initialized")
    
    def get_manager(self) -> PipelineManager:
        """Get pipeline manager."""
        return self.manager
    
    def get_executor(self) -> PipelineExecutor:
        """Get pipeline executor."""
        return self.executor
    
    def execute_pipeline(self, pipeline_name: str, additional_context: Optional[Dict] = None, 
                        stack_manifest: Optional[Dict] = None):
        """Execute a pipeline directly (legacy interface)."""
        try:
            pipeline = self.manager.get_pipeline(pipeline_name)
            if pipeline:
                if self.logger:
                    self.logger.info(f"Executing pipeline: {pipeline_name} with context: {additional_context}")
                
                # Create synthetic pipeline request event
                pipeline_event = PipelineRequestedEvent(
                    event_type=EventType.PIPELINE_REQUESTED,
                    source_component="pipeline_engine_legacy",
                    pipeline_name=pipeline_name,
                    execution_context=additional_context or {},
                    stack_manifest=stack_manifest or {}
                )
                
                self.executor._execute_pipeline_internal(pipeline_event)
            else:
                if self.logger:
                    self.logger.warning(f"No pipeline found with name: {pipeline_name}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error executing pipeline: {e}")