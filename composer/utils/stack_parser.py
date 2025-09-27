"""
Stack Parser Utility for Eclipse Muto Composer

This module provides utilities for parsing different stack payload formats
including direct stack payloads, solution manifests, and archive formats.
"""

import json
import base64
import gzip
import io
from typing import Optional, Dict, Any
import logging


class StackParser:
    """
    Utility class for parsing and extracting stack definitions from various payload formats.
    
    Supports:
    - Direct stack JSON payloads (new format)
    - Archive stack payloads (new format) 
    - Solution manifest payloads with embedded stack components (old format)
    - Base64 encoded and compressed stack data
    """
    
    def __init__(self, logger: Optional[logging.Logger] = None):
        """
        Initialize the StackParser.
        
        Args:
            logger: Optional logger instance for debugging and error reporting
        """
        self.logger = logger or logging.getLogger(__name__)
    
    def parse_payload(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Main entry point for parsing stack payloads.
        
        Args:
            payload: The payload dictionary to parse
            
        Returns:
            Parsed stack dictionary if successful, original payload if has "value" key, None otherwise
            
        Rules:
        - If payload has a "value" key, return payload as-is (no parsing needed)
        - Otherwise, attempt to extract stack from various formats
        """
        if not isinstance(payload, dict):
            self.logger.warning("Payload is not a dictionary")
            return None
            
        # If payload has a value key, return as-is (existing behavior)
        if "value" in payload:
            self.logger.debug("Payload contains 'value' key, returning as-is")
            return payload
            
        # Try different parsing strategies in order of preference
        # 1. Try new direct stack format (stack.json style)
        stack = self._try_direct_stack_json(payload)
        if stack:
            return stack
            
        # 2. Try new archive format (stack-archive-example.json style)
        stack = self._try_archive_format(payload)
        if stack:
            return stack
            
        # 3. Try old solution manifest format (talker-listener-archive-solution.json style)
        stack = self._try_solution_manifest(payload)
        if stack:
            return stack

        #if payload is dictionary and has node or composable return the payloadas is
        if isinstance(payload, dict) and ("node" in payload or "composable" in payload):
            self.logger.debug("Payload contains 'node' or 'composable', returning as-is")
            return payload
        
        self.logger.warning("Could not parse stack from payload")
        return None
    
    def _try_direct_stack_json(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Try to parse payload as a direct stack/json definition.
        
        Direct stack format example (stack.json):
        {
            "metadata": {
                "name": "...",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [...],
                "composable": [...]
            }
        }
        """
        metadata = payload.get("metadata", {})
        content_type = metadata.get("content_type")
        
        if content_type == "stack/json":
            return payload
                        
        return None
    
    def _try_archive_format(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Try to parse payload as an archive format stack.
        
        Archive format example (stack-archive-example.json):
        {
            "metadata": {
                "name": "...",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "base64_encoded_compressed_data",
                "properties": { ... }
            }
        }
        """
        metadata = payload.get("metadata", {})
        content_type = metadata.get("content_type")
        
        if content_type == "stack/archive":
            return payload
                    
        return None
    
    def _try_solution_manifest(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Try to parse payload as a solution manifest with embedded stack components.
        
        This handles the existing _extract_stack_from_solution logic for the old format.
        Solution manifest format (talker-listener-archive-solution.json):
        {
            "spec": {
                "components": [
                    {
                        "name": "...",
                        "type": "muto-agent",
                        "properties": {
                            "type": "stack",
                            "data": "base64_encoded_data"
                        }
                    }
                ]
            }
        }
        """
        spec = payload.get("spec")
        if not isinstance(spec, dict):
            return None

        components = spec.get("components", [])
        if not isinstance(components, list):
            return None

        for component in components:
            properties = component.get("properties", {})
            if properties.get("type") != "stack":
                continue
                
            data_b64 = properties.get("data")
            if not data_b64:
                continue

            try:
                stack = self._decode_base64_stack(data_b64)
                if stack:
                    self.logger.debug(f"Extracted stack from solution component '{component.get('name', '')}'")
                    return stack
            except Exception as exc:
                self.logger.warning(
                    f"Failed to decode stack component '{component.get('name', '')}' from solution: {exc}"
                )
                
        return None
    
    def _decode_base64_stack(self, data_b64: str) -> Optional[Dict[str, Any]]:
        """
        Decode base64 encoded stack data, handling compression if present.
        
        Args:
            data_b64: Base64 encoded stack data
            
        Returns:
            Decoded stack dictionary if successful, None otherwise
        """
        try:
            raw = base64.b64decode(data_b64)
            
            # Handle potential gzip compression
            while True:
                try:
                    stack_json = raw.decode("utf-8")
                    stack_dict = json.loads(stack_json)
                    return stack_dict
                except (UnicodeDecodeError, json.JSONDecodeError):
                    # Check for gzip compression using magic numbers
                    if len(raw) > 2 and raw[0] == 0x1F and raw[1] == 0x8B:  # gzip magic numbers
                        with gzip.GzipFile(fileobj=io.BytesIO(raw)) as gz:
                            raw = gz.read()
                        continue
                    raise
                    
        except (ValueError, json.JSONDecodeError, OSError) as exc:
            self.logger.warning(f"Failed to decode base64 stack data: {exc}")
            return None
    
    def validate_stack(self, stack: Dict[str, Any]) -> bool:
        """
        Validate that the parsed result is a valid stack definition.
        
        Args:
            stack: Stack dictionary to validate
            
        Returns:
            True if valid stack, False otherwise
        """
        if not isinstance(stack, dict):
            return False
            
        # Basic validation - at least one of these should be present for a valid stack
        required_fields = ["node", "composable", "launch_description_source", "artifact", "archive_properties"]
        optional_fields = ["stackId", "name", "context", "on_start", "on_kill", "content_type"]
        
        has_required = any(field in stack for field in required_fields)
        has_structure = any(field in stack for field in required_fields + optional_fields)
        
        return has_required or has_structure


def create_stack_parser(logger: Optional[logging.Logger] = None) -> StackParser:
    """
    Factory function to create a StackParser instance.
    
    Args:
        logger: Optional logger instance
        
    Returns:
        StackParser instance
    """
    return StackParser(logger)