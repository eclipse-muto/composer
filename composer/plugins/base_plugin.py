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
from threading import Event

import rclpy
from rclpy.node import Node
from composer.utils.stack_parser import StackParser
from muto_msgs.srv import CoreTwin

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
        self._stack_definition_client = self.create_client(
            CoreTwin, "/muto/core_twin/get_stack_definition"
        )
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
        if not request:
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

    def _fetch_stack_manifest(self, stack_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a stack manifest from CoreTwin using the provided stack ID.
        """
        if not stack_id:
            return None

        try:
            if not self._stack_definition_client:
                return None

            if not self._stack_definition_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warning(
                    "CoreTwin get_stack_definition service is not available."
                )
                return None

            request = CoreTwin.Request()
            request.input = stack_id

            future = self._stack_definition_client.call_async(request)
            result_holder = {"manifest": None, "event": Event()}

            def _cb(fut):
                self._handle_twin_response(fut, result_holder, stack_id)

            future.add_done_callback(_cb)

            completed = result_holder["event"].wait(timeout=5.0)
            if not completed:
                self.get_logger().warning(
                    f"Timeout reached while waiting for stack manifest: {stack_id}"
                )
                return None

            return result_holder["manifest"]

        except Exception as exc:
            self.get_logger().error(
                f"Error fetching stack manifest for stackId '{stack_id}': {exc}"
            )

        return None

    def _handle_twin_response(self, future: rclpy.Future, holder: Dict[str, Any], stack_id: str):
        """
        Callback to process CoreTwin responses without blocking execution.
        """
        try:
            result = future.result()
            if not result or not getattr(result, "output", None):
                self.get_logger().warning(
                    f"CoreTwin returned an empty manifest for stackId '{stack_id}'."
                )
                holder["manifest"] = None
            else:
                try:
                    holder["manifest"] = json.loads(result.output)
                except json.JSONDecodeError as json_err:
                    self.get_logger().error(
                        f"Failed to decode manifest for stackId '{stack_id}': {json_err}"
                    )
                    holder["manifest"] = None
        except Exception as exc:
            self.get_logger().error(
                f"Error processing manifest response for stackId '{stack_id}': {exc}"
            )
            holder["manifest"] = None
        finally:
            holder["event"].set()

    def _is_manifest_payload(self, stack_dict: Dict[str, Any]) -> bool:
        """
        Determine whether the provided dictionary already looks like a full stack manifest.
        """
        if not isinstance(stack_dict, dict):
            return False

        # Stack references usually only contain id/state. Anything else is treated as a manifest.
        reference_keys = {"stackId", "state"}
        return not set(stack_dict.keys()).issubset(reference_keys)

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
            if isinstance(stack_string, dict):
                parsed = stack_string
            else:
                parsed = json.loads(stack_string)

            if not isinstance(parsed, dict):
                self.get_logger().warning(
                    f"Stack string parsed to non-dict type: {type(parsed)}"
                )
                return None

            if self._is_manifest_payload(parsed):
                return parsed

            # Support payloads that wrap stackId under a value field
            stack_id = None
            if "stackId" in parsed:
                stack_id = parsed.get("stackId")
            elif "value" in parsed and isinstance(parsed["value"], dict):
                stack_id = parsed["value"].get("stackId")

            if not stack_id:
                self.get_logger().warning("Stack payload did not contain a stackId.")
                return None

            manifest = self._fetch_stack_manifest(stack_id)
            if manifest:
                return manifest

            # Fallback to parsed data if manifest retrieval fails
            self.get_logger().warning(
                f"Falling back to stack reference for stackId '{stack_id}'."
            )
            return parsed

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
    
