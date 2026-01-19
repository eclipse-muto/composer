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

import hashlib
import json
import os
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

from rclpy.node import Node
from composer.utils.stack_parser import StackParser

WORKSPACES_PATH = os.path.join("/tmp", "muto", "muto_workspaces")
ARTIFACT_STATE_FILE = ".muto_artifact.json"


class StackOperation(Enum):
    """Enumeration of stack operations."""
    START = "start"
    KILL = "kill"
    APPLY = "apply"
    PROVISION = "provision"
    COMPOSE = "compose"


@dataclass
class StackContext:
    """Context object passed during double dispatch."""
    stack_data: Dict[str, Any]
    metadata: Dict[str, Any]
    operation: StackOperation
    name: Optional[str] = None
    logger: Optional[Any] = None
    # Additional context data can be added here
    workspace_path: Optional[str] = None
    launcher: Optional[Any] = None
    hash: str = None



class BasePlugin(Node):
    """Base class for stack plugins implementing the Plugin interface."""

    def __init__(self, node_name: str):
        from composer.stack_handlers.registry import StackTypeRegistry
        Node.__init__(self, node_name)
        self.stack_registry = StackTypeRegistry(self, self.get_logger())
        self.stack_registry.discover_and_register_handlers()
        self.stack_parser = StackParser(self.get_logger())

       
    # Plugin interface implementation - single accept method
   
    def _get_stack_name(self, stack_dict):
        """
        Get the stack name from a stack dictionary, checking metadata.name first, then name, then defaulting to 'default'.
        
        Args:
            stack_dict (dict): The stack dictionary
            
        Returns:
            str: The stack name
        """
        if not stack_dict or not isinstance(stack_dict, dict):
            return "default"
            
        metadata = stack_dict.get("metadata", {})
        return metadata.get("name", stack_dict.get("name", "default"))
 
            
    def find_file(self, ws_path: str, file_name: str) -> Optional[str]:
        """
        Helper method to find a file in the workspace path.

        Args:
            ws_path (str): The workspace path to search in.
            file_name (str): The name of the file to find.

        Returns:
            Optional[str]: The full path to the file if found, otherwise None.
        """
        self.get_logger().info(f"Searching for {file_name} under {ws_path}")

        candidate = os.path.join(ws_path, file_name)
        if os.path.isfile(candidate):
            self.get_logger().info(f"Found file directly: {candidate}")
            return candidate

        basename = os.path.basename(file_name)
        for root, _, files in os.walk(ws_path):
            if basename in files:
                found_path = os.path.join(root, basename)
                self.get_logger().info(f"Found file: {found_path}")
                return found_path
        self.get_logger().warning(f"File '{file_name}' not found under '{ws_path}'.")
        return None

       
    def find_stack_handler(self, request):
        """
        Get the stack handler for the current context.
        Validates the stack manifest before processing.
        """
        if not request.input.current.stack:
            return None, None

        current_stack = self._safely_parse_stack(request.input.current.stack)
        # encode str and sha256 hash of the stack contents
        # Handle both real strings and test mocks
        if isinstance(request.input.current.stack, str):
            hash = hashlib.sha256(request.input.current.stack.encode()).hexdigest()
        else:
            hash = None
        try:
            if current_stack:
                # Validate stack manifest before processing
                if not self._validate_stack_manifest(current_stack):
                    self.get_logger().error("Stack manifest validation failed")
                    return None, None

                # Get appropriate handler from registry
                handler = self.stack_registry.get_handler(current_stack)
                if handler:
                    # Get stack name for workspace path
                    stack_name = self._get_stack_name(current_stack)
                    workspace_path = os.path.join(
                        WORKSPACES_PATH,
                        stack_name.replace(" ", "_"),
                    )
                    
                    # Create context for double dispatch
                    context = StackContext(
                        stack_data=current_stack,
                        metadata=current_stack.get("metadata", {}),
                        operation=StackOperation.START,
                        name=current_stack.get("name", "default"),
                        logger=self.get_logger(),
                        workspace_path=workspace_path,
                        hash=hash
                    )
                    return handler, context
        except Exception as e:
            self.get_logger().error(f"Error creating stack_handler: {e}")
            return None,None
        return None,None

    def _safely_parse_stack(self, stack_string):
        """
        Safely parse a stack string to JSON. Returns dictionary if valid JSON, None otherwise.
        
        Args:
            stack_string (str): The stack string to parse
            
        Returns:
            dict or None: Parsed JSON dictionary or None if parsing fails
        """
        if not stack_string:
            return None
            
        try:
            parsed = json.loads(stack_string)
            if isinstance(parsed, dict):
                return parsed
            else:
                self.get_logger().warning(f"Stack string parsed to non-dict type: {type(parsed)}")
                return None
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warning(f"Failed to parse stack string as JSON: {e}")
            return None

    def _validate_stack_manifest(self, stack: Dict[str, Any]) -> bool:
        """
        Validate that the stack manifest is well-formed before processing.

        Args:
            stack: The parsed stack dictionary

        Returns:
            True if valid, False otherwise
        """
        if not self.stack_parser.validate_stack(stack):
            self.get_logger().warning("Stack manifest failed basic validation - missing required structure")
            return False

        # Additional validation: check for content_type consistency if metadata present
        metadata = stack.get("metadata", {})
        content_type = metadata.get("content_type")

        if content_type:
            # Validate that the content matches declared type
            if content_type == "stack/archive":
                if not stack.get("launch"):
                    self.get_logger().warning(
                        "Stack declares stack/archive but missing launch section"
                    )
                    return False
            elif content_type == "stack/json":
                if not stack.get("launch"):
                    self.get_logger().warning(
                        "Stack declares stack/json but missing launch section"
                    )
                    return False

        return True


class StackTypeHandler(ABC):
    """
    Abstract base class for stack type handlers.
    Simplified with double dispatch pattern.
    """
    
    @abstractmethod
    def can_handle(self, payload: Dict[str, Any]) -> bool:
        """Determine if this handler can process the given payload."""
        pass
    
    @abstractmethod
    def apply_to_plugin(self, plugin: BasePlugin, context: StackContext, request, response) -> any:
        """
        Apply this stack type to a plugin using double dispatch.
        The handler processes the type-specific logic for STACK operations.
        """
        pass
    