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

from typing import Dict, Any
from composer.plugins.base_plugin import BasePlugin, StackTypeHandler, StackContext, StackOperation
from composer.model.stack import Stack


class LegacyStackHandler(StackTypeHandler):
    """Handler for legacy launch/json format stacks."""
    
    def __init__(self, logger=None):
        self.logger = logger
    
    def can_handle(self, payload: Dict[str, Any]) -> bool:
        """
        Check if payload matches legacy format:
        - No proper metadata.content_type (not a properly defined solution)
        - Has launch-related structure (node, composable, or legacy patterns)
        """
        if not isinstance(payload, dict):
            return False
            
        metadata = payload.get("metadata", {})
        content_type = metadata.get("content_type")
        
        # If there's a content_type, it's a properly defined solution
        if content_type:
            return False
        
        # Check for legacy launch structures
        has_nodes = bool(payload.get("node") or payload.get("composable"))
        has_launch = bool(payload.get("launch"))
        has_legacy_patterns = bool(
            payload.get("launch_description_source") or
            (payload.get("on_start") and payload.get("on_kill"))
        )
        
        return has_nodes or has_launch or has_legacy_patterns
    
    def apply_to_plugin(self, plugin: BasePlugin, context: StackContext, request, response) -> bool:
        """Double dispatch: delegate to plugin's accept method."""

        try:
            if context.operation == StackOperation.PROVISION:
                return True
            elif context.operation == StackOperation.START:
                return self._start_legacy(context)
            elif context.operation == StackOperation.KILL:
                return self._kill_legacy(context)
            elif context.operation == StackOperation.APPLY:
                return self._apply_legacy(context)
            else:
                if self.logger:
                    self.logger.warning(f"Unsupported operation for legacy stack: {context.operation}")
                return False
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error processing legacy stack operation: {e}")
            return False
    
    
    def _start_legacy(self, context: StackContext) -> bool:
        """Start a legacy stack."""
        try:
            # Check if it's a script-based legacy stack (on_start/on_kill)
            if context.stack_data.get("on_start") and context.stack_data.get("on_kill"):
                # Script-based legacy stack - let plugin handle this
                if self.logger:
                    self.logger.info("Legacy script-based stack start delegated to plugin")
                return True
            
            # Otherwise, try to use Stack model with node/composable arrays
            if context.stack_data.get("node") or context.stack_data.get("composable"):
                stack = Stack(manifest=context.stack_data)
                stack.launch(context.launcher)
                return True
            
            # Check if there's a launch structure
            launch_data = context.stack_data.get("launch")
            if launch_data:
                stack = Stack(manifest=launch_data)
                stack.launch(context.launcher)
                return True
            
            if self.logger:
                self.logger.warning("No recognizable launch structure in legacy stack")
            return False
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error starting legacy stack: {e}")
            return False
    
    def _kill_legacy(self, context: StackContext) -> bool:
        """Kill a legacy stack."""
        try:
            # Check if it's a script-based legacy stack
            if context.stack_data.get("on_start") and context.stack_data.get("on_kill"):
                # Script-based legacy stack - let plugin handle this
                if self.logger:
                    self.logger.info("Legacy script-based stack kill delegated to plugin")
                return True
            
            # Otherwise, try to use Stack model
            if context.stack_data.get("node") or context.stack_data.get("composable"):
                stack = Stack(manifest=context.stack_data)
                stack.kill()
                return True
            
            # Check if there's a launch structure
            launch_data = context.stack_data.get("launch")
            if launch_data:
                stack = Stack(manifest=launch_data)
                stack.kill()
                return True
            
            if self.logger:
                self.logger.warning("No recognizable launch structure in legacy stack for kill")
            return False
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error killing legacy stack: {e}")
            return False
    
    def _apply_legacy(self, context: StackContext) -> bool:
        """Apply a legacy stack configuration."""
        try:
            # Legacy stacks with node/composable support apply
            if context.stack_data.get("node") or context.stack_data.get("composable"):
                stack = Stack(manifest=context.stack_data)
                stack.apply(context.launcher)
                return True
            
            # Check if there's a launch structure
            launch_data = context.stack_data.get("launch")
            if launch_data:
                stack = Stack(manifest=launch_data)
                stack.apply(context.launcher)
                return True
            
            # Script-based legacy stacks don't support apply - no-op
            if self.logger:
                self.logger.info("Legacy script-based stacks do not support apply (no-op)")
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error applying legacy stack: {e}")
            return False
