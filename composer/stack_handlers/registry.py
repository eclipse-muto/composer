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

from typing import Dict, List, Optional

from rclpy.node import Node
from composer.plugins.base_plugin import StackTypeHandler


class StackTypeRegistry:
    """Registry for stack type handlers."""
    
    def __init__(self, node: Node, logger=None):
        self.handlers: List[StackTypeHandler] = []
        if node:
            node.declare_parameter("ignored_packages", [""])
            self.ignored_packages = [
                pkg
                for pkg in node.get_parameter("ignored_packages")
                .get_parameter_value()
                .string_array_value
                if pkg
            ]
        self.logger = logger
    
    def register_handler(self, handler: StackTypeHandler) -> None:
        """Register a stack type handler."""
        self.handlers.append(handler)
        
    def get_handler(self, payload: Dict) -> Optional[StackTypeHandler]:
        """
        Find the appropriate handler for a payload.
        
        Priority:
        1. Properly defined solutions with metadata.content_type
        2. Legacy format validation
        """
        if not isinstance(payload, dict):
            self.logger.warning("Invalid payload type, expected dict")
            return None
        
        # Check if payload has proper metadata structure
        metadata = payload.get("metadata", {})
        content_type = metadata.get("content_type")
        
        if not content_type:
            self.logger.debug("No content_type found, checking for legacy format")
        
        # Try each registered handler
        for handler in self.handlers:
            if handler.can_handle(payload):
                handler_name = handler.__class__.__name__
                self.logger.debug(f"Selected handler: {handler_name}")
                return handler
        
        self.logger.warning("No handler found for payload")
        return None
    
    def discover_and_register_handlers(self) -> None:
        """Automatically discover and register all available handlers."""
        from .json_handler import JsonStackHandler
        from .archive_handler import ArchiveStackHandler
        from .legacy_handler import LegacyStackHandler
        
        # Register in priority order
        self.register_handler(JsonStackHandler(self.logger))
        self.register_handler(ArchiveStackHandler(self.logger, self.ignored_packages))
        self.register_handler(LegacyStackHandler(self.logger))
