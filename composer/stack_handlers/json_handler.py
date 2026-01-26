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
from composer.plugins.base_plugin import StackTypeHandler, BasePlugin, StackContext, StackOperation
from composer.model.stack import Stack
from composer.workflow.launcher import Ros2LaunchParent


class JsonStackHandler(StackTypeHandler):
    """Handler for stack/json type stacks."""
    
    def __init__(self, logger=None):
        self.logger = logger
        self.is_up_to_date = False
        self.managed_launchers = {}

    
    def can_handle(self, payload: Dict[str, Any]) -> bool:
        """Check for stack/json content_type in properly defined solution."""
        if not isinstance(payload, dict):
            return False
        metadata = payload.get("metadata", {})
        content_type = metadata.get("content_type", "")
        return content_type == "stack/json"
    
    def apply_to_plugin(self, plugin: BasePlugin, context: StackContext, request, response) -> bool:
        """Double dispatch: delegate to plugin's accept method."""
        
        if context.operation == StackOperation.PROVISION:
            return self._provision_json(context, plugin)
        elif context.operation == StackOperation.START:
            return self._start_json(context, plugin)
        elif context.operation == StackOperation.KILL:
            return self._kill_json(context, plugin)
        elif context.operation == StackOperation.APPLY:
            return self._apply_json(context, plugin)
        else:
            self.logger.warning(f"Unsupported operation for JSON stack: {context.operation}")
            return False

    def _provision_json(self, context: StackContext, plugin: BasePlugin) -> bool:
        try:
            # JSON stacks don't require workspace provisioning - they're pure launch configs
            # Mark as up-to-date since there's no artifact to provision
            self.is_up_to_date = True
            self.logger.info("JSON stack loaded - no workspace provisioning needed.")
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error in provisioning JSON stack: {e}")
            return False

    def _start_json(self, context: StackContext, plugin: BasePlugin) -> bool:
            
        # JSON stacks support launch operations
        # For stack/json, the launch data is inside the manifest
        
        self._kill_json(context, plugin)

        launcher = Ros2LaunchParent([])        
        launch_data = context.stack_data.get("launch")
        if not launch_data:
            self.logger.error("No 'launch' section found in stack/json manifest")
            return False
        stack = Stack(manifest=launch_data)
        stack.launch(launcher)
        self.managed_launchers[context.hash] = launcher
        return True
    
    def _kill_json(self, context: StackContext, plugin: BasePlugin) -> bool:
            
        # JSON stacks support launch operations
        # For stack/json, the launch data is inside the manifest
        #launch_data = context.stack_data.get("launch")
        #if not launch_data:
        #    self.logger.error("No 'launch' section found in stack/json manifest")
        #    return False
        #stack = Stack(manifest=launch_data)
        launcher = self.managed_launchers.get(context.hash, None)
        if launcher:
            launcher.kill()
            self.managed_launchers.pop(context.hash, None)
        return True

    def _apply_json(self, context: StackContext, plugin: BasePlugin) -> bool:

        # JSON stacks support launch operations
        # For stack/json, the launch data is inside the manifest
        self._kill_json(context, plugin)

        launcher = Ros2LaunchParent([])
        launch_data = context.stack_data.get("launch")
        if not launch_data:
            self.logger.error("No 'launch' section found in stack/json manifest")
            return False
        stack = Stack(manifest=launch_data)
        stack.apply(launcher)
        self.managed_launchers[context.hash] = launcher
        return True

