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
State persistence module for Muto Composer.

Provides persistent storage of stack deployment states to enable
rollback to previous versions when deployments fail.
"""

import json
import os
import copy
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Dict, Any, Optional
from enum import Enum

from composer.utils.paths import get_state_path


class DeploymentStatus(Enum):
    """Deployment status enumeration."""
    PENDING = "pending"
    DEPLOYING = "deploying"
    RUNNING = "running"
    FAILED = "failed"
    ROLLED_BACK = "rolled_back"
    STOPPED = "stopped"


@dataclass
class StackState:
    """Represents the persistent state of a stack deployment."""
    stack_id: str = ""
    stack_name: str = ""
    current_version: str = ""
    previous_version: str = ""
    current_stack: Optional[Dict[str, Any]] = None
    previous_stack: Optional[Dict[str, Any]] = None
    status: str = DeploymentStatus.PENDING.value
    deployed_at: str = ""
    last_updated: str = ""
    error_message: str = ""
    rollback_count: int = 0

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "stack_id": self.stack_id,
            "stack_name": self.stack_name,
            "current_version": self.current_version,
            "previous_version": self.previous_version,
            "current_stack": self.current_stack,
            "previous_stack": self.previous_stack,
            "status": self.status,
            "deployed_at": self.deployed_at,
            "last_updated": self.last_updated,
            "error_message": self.error_message,
            "rollback_count": self.rollback_count,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "StackState":
        """Create StackState from dictionary."""
        return cls(
            stack_id=data.get("stack_id", ""),
            stack_name=data.get("stack_name", ""),
            current_version=data.get("current_version", ""),
            previous_version=data.get("previous_version", ""),
            current_stack=data.get("current_stack"),
            previous_stack=data.get("previous_stack"),
            status=data.get("status", DeploymentStatus.PENDING.value),
            deployed_at=data.get("deployed_at", ""),
            last_updated=data.get("last_updated", ""),
            error_message=data.get("error_message", ""),
            rollback_count=data.get("rollback_count", 0),
        )


class StatePersistence:
    """
    Manages persistent storage of stack deployment states.

    Stores state files at ~/.muto/state/<stack_name>/state.json
    to enable rollback to previous versions on deployment failure.

    Also maintains a global active deployment state at ~/.muto/state/_active/state.json
    to track the currently running stack across different stack names, enabling
    rollback to a different stack when deployment fails.
    """

    STATE_FILENAME = "state.json"
    ACTIVE_STATE_DIR = "_active"

    def __init__(self, logger=None):
        self.logger = logger
        self._state_root = get_state_path()
        self._ensure_state_directory()

    def _ensure_state_directory(self) -> None:
        """Ensure the state root directory exists."""
        os.makedirs(self._state_root, exist_ok=True)

    def _get_stack_state_dir(self, stack_name: str) -> str:
        """Get the state directory for a specific stack."""
        # Sanitize stack name for filesystem
        safe_name = stack_name.replace(" ", "_").replace("/", "_").replace(":", "_")
        return os.path.join(self._state_root, safe_name)

    def _get_state_file_path(self, stack_name: str) -> str:
        """Get the state file path for a specific stack."""
        return os.path.join(self._get_stack_state_dir(stack_name), self.STATE_FILENAME)

    def _get_version_from_stack(self, stack: Optional[Dict[str, Any]]) -> str:
        """Extract version from stack metadata."""
        if not stack:
            return ""
        metadata = stack.get("metadata", {})
        return metadata.get("version", metadata.get("name", "unknown"))

    def _get_stack_id_from_stack(self, stack: Optional[Dict[str, Any]]) -> str:
        """Extract stack ID from stack."""
        if not stack:
            return ""
        # Try different locations for stack ID
        if "stackId" in stack:
            return stack["stackId"]
        metadata = stack.get("metadata", {})
        if "id" in metadata:
            return metadata["id"]
        if "name" in metadata:
            return metadata["name"]
        return stack.get("name", "")

    def load_state(self, stack_name: str) -> Optional[StackState]:
        """
        Load the persisted state for a stack.

        Args:
            stack_name: Name of the stack

        Returns:
            StackState if found, None otherwise
        """
        state_path = self._get_state_file_path(stack_name)

        if not os.path.exists(state_path):
            if self.logger:
                self.logger.debug(f"No state file found for stack: {stack_name}")
            return None

        try:
            with open(state_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            state = StackState.from_dict(data)
            if self.logger:
                self.logger.debug(f"Loaded state for stack: {stack_name}, status: {state.status}")
            return state
        except (OSError, json.JSONDecodeError) as e:
            if self.logger:
                self.logger.warning(f"Failed to load state for {stack_name}: {e}")
            return None

    def save_state(self, stack_name: str, state: StackState) -> bool:
        """
        Save the state for a stack.

        Args:
            stack_name: Name of the stack
            state: StackState to save

        Returns:
            True if successful, False otherwise
        """
        state_dir = self._get_stack_state_dir(stack_name)
        state_path = self._get_state_file_path(stack_name)

        try:
            os.makedirs(state_dir, exist_ok=True)
            state.last_updated = datetime.utcnow().isoformat() + "Z"

            with open(state_path, "w", encoding="utf-8") as f:
                json.dump(state.to_dict(), f, indent=2)

            if self.logger:
                self.logger.debug(f"Saved state for stack: {stack_name}")
            return True
        except OSError as e:
            if self.logger:
                self.logger.error(f"Failed to save state for {stack_name}: {e}")
            return False

    def get_previous_stack(self, stack_name: str) -> Optional[Dict[str, Any]]:
        """
        Get the previous stack definition for rollback.

        Args:
            stack_name: Name of the stack

        Returns:
            Previous stack definition if available, None otherwise
        """
        state = self.load_state(stack_name)
        if state and state.previous_stack:
            if self.logger:
                self.logger.info(f"Retrieved previous stack for rollback: {stack_name}")
            return state.previous_stack
        return None

    def mark_deployment_started(self, stack_name: str, next_stack: Dict[str, Any]) -> bool:
        """
        Mark deployment as started and save current stack as previous.

        This should be called before starting a new deployment to preserve
        the current stack for potential rollback.

        Args:
            stack_name: Name of the stack
            next_stack: The new stack being deployed

        Returns:
            True if successful, False otherwise
        """
        # Load existing state or create new
        state = self.load_state(stack_name)
        if state is None:
            state = StackState(stack_name=stack_name)

        # Move current to previous before deployment
        if state.current_stack:
            state.previous_stack = copy.deepcopy(state.current_stack)
            state.previous_version = state.current_version

        # Set new deployment info
        state.stack_id = self._get_stack_id_from_stack(next_stack)
        state.current_version = self._get_version_from_stack(next_stack)
        state.current_stack = copy.deepcopy(next_stack)
        state.status = DeploymentStatus.DEPLOYING.value
        state.deployed_at = datetime.utcnow().isoformat() + "Z"
        state.error_message = ""

        if self.logger:
            self.logger.info(
                f"Deployment started for {stack_name}: "
                f"version {state.current_version}, previous: {state.previous_version}"
            )

        return self.save_state(stack_name, state)

    def mark_deployment_completed(self, stack_name: str) -> bool:
        """
        Mark deployment as successfully completed.

        Args:
            stack_name: Name of the stack

        Returns:
            True if successful, False otherwise
        """
        state = self.load_state(stack_name)
        if state is None:
            if self.logger:
                self.logger.warning(f"No state found to mark completed: {stack_name}")
            return False

        state.status = DeploymentStatus.RUNNING.value
        state.error_message = ""

        if self.logger:
            self.logger.info(f"Deployment completed for {stack_name}: version {state.current_version}")

        return self.save_state(stack_name, state)

    def mark_deployment_failed(self, stack_name: str, error: str = "") -> bool:
        """
        Mark deployment as failed.

        Args:
            stack_name: Name of the stack
            error: Error message describing the failure

        Returns:
            True if successful, False otherwise
        """
        state = self.load_state(stack_name)
        if state is None:
            state = StackState(stack_name=stack_name)

        state.status = DeploymentStatus.FAILED.value
        state.error_message = error

        if self.logger:
            self.logger.error(f"Deployment failed for {stack_name}: {error}")

        return self.save_state(stack_name, state)

    def mark_rollback_completed(self, stack_name: str) -> bool:
        """
        Mark that a rollback was completed successfully.

        Swaps current and previous stacks to reflect the rollback.

        Args:
            stack_name: Name of the stack

        Returns:
            True if successful, False otherwise
        """
        state = self.load_state(stack_name)
        if state is None:
            if self.logger:
                self.logger.warning(f"No state found for rollback completion: {stack_name}")
            return False

        # Swap current with previous (rollback)
        failed_stack = state.current_stack
        failed_version = state.current_version

        state.current_stack = state.previous_stack
        state.current_version = state.previous_version
        state.previous_stack = failed_stack
        state.previous_version = failed_version
        state.status = DeploymentStatus.ROLLED_BACK.value
        state.rollback_count += 1

        if self.logger:
            self.logger.info(
                f"Rollback completed for {stack_name}: "
                f"restored version {state.current_version}"
            )

        return self.save_state(stack_name, state)

    def can_rollback(self, stack_name: str) -> bool:
        """
        Check if rollback is possible for a stack.

        Args:
            stack_name: Name of the stack

        Returns:
            True if a previous stack exists for rollback
        """
        state = self.load_state(stack_name)
        return state is not None and state.previous_stack is not None

    def get_all_stack_states(self) -> Dict[str, StackState]:
        """
        Get states for all tracked stacks.

        Returns:
            Dictionary mapping stack names to their states
        """
        states = {}

        if not os.path.exists(self._state_root):
            return states

        try:
            for entry in os.listdir(self._state_root):
                entry_path = os.path.join(self._state_root, entry)
                if os.path.isdir(entry_path) and entry != self.ACTIVE_STATE_DIR:
                    state = self.load_state(entry)
                    if state:
                        states[entry] = state
        except OSError as e:
            if self.logger:
                self.logger.warning(f"Failed to list stack states: {e}")

        return states

    # =========================================================================
    # Global Active Deployment State Methods
    # =========================================================================
    # These methods track the currently active deployment across ALL stack names,
    # enabling rollback to a different stack when deployment fails.

    def _get_active_state_path(self) -> str:
        """Get the path to the global active deployment state file."""
        return os.path.join(self._state_root, self.ACTIVE_STATE_DIR, self.STATE_FILENAME)

    def load_active_state(self) -> Optional[StackState]:
        """
        Load the global active deployment state.

        This tracks which stack is currently running, regardless of stack name.

        Returns:
            StackState if found, None otherwise
        """
        state_path = self._get_active_state_path()

        if not os.path.exists(state_path):
            if self.logger:
                self.logger.debug("No active deployment state file found")
            return None

        try:
            with open(state_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            state = StackState.from_dict(data)
            if self.logger:
                self.logger.debug(f"Loaded active state: {state.stack_name}, status: {state.status}")
            return state
        except (OSError, json.JSONDecodeError) as e:
            if self.logger:
                self.logger.warning(f"Failed to load active state: {e}")
            return None

    def save_active_state(self, state: StackState) -> bool:
        """
        Save the global active deployment state.

        Args:
            state: StackState to save

        Returns:
            True if successful, False otherwise
        """
        state_dir = os.path.join(self._state_root, self.ACTIVE_STATE_DIR)
        state_path = self._get_active_state_path()

        try:
            os.makedirs(state_dir, exist_ok=True)
            state.last_updated = datetime.utcnow().isoformat() + "Z"

            with open(state_path, "w", encoding="utf-8") as f:
                json.dump(state.to_dict(), f, indent=2)

            if self.logger:
                self.logger.debug(f"Saved active state: {state.stack_name}")
            return True
        except OSError as e:
            if self.logger:
                self.logger.error(f"Failed to save active state: {e}")
            return False

    def mark_active_deployment_started(self, next_stack: Dict[str, Any]) -> bool:
        """
        Mark a new deployment as started in the global active state.

        Saves the currently running stack as 'previous' to enable rollback
        to a different stack if the new deployment fails.

        Args:
            next_stack: The new stack being deployed

        Returns:
            True if successful, False otherwise
        """
        # Load existing active state or create new
        state = self.load_active_state()
        if state is None:
            state = StackState(stack_name="_active")

        # Move current to previous before deployment (this is the key for cross-stack rollback)
        if state.current_stack:
            state.previous_stack = copy.deepcopy(state.current_stack)
            state.previous_version = state.current_version
            if self.logger:
                prev_name = state.previous_stack.get("metadata", {}).get("name", "unknown")
                self.logger.info(f"Saved previous stack for rollback: {prev_name}")

        # Set new deployment info
        stack_name = self._get_stack_name_from_stack(next_stack)
        state.stack_id = self._get_stack_id_from_stack(next_stack)
        state.stack_name = stack_name or "_active"
        state.current_version = self._get_version_from_stack(next_stack)
        state.current_stack = copy.deepcopy(next_stack)
        state.status = DeploymentStatus.DEPLOYING.value
        state.deployed_at = datetime.utcnow().isoformat() + "Z"
        state.error_message = ""

        if self.logger:
            self.logger.info(
                f"Active deployment started: {stack_name} v{state.current_version}, "
                f"previous: {state.previous_version}"
            )

        return self.save_active_state(state)

    def _get_stack_name_from_stack(self, stack: Optional[Dict[str, Any]]) -> Optional[str]:
        """Extract stack name from stack definition."""
        if not stack:
            return None
        metadata = stack.get("metadata", {})
        return metadata.get("name") or stack.get("name")

    def mark_active_deployment_completed(self) -> bool:
        """
        Mark the active deployment as successfully completed.

        Returns:
            True if successful, False otherwise
        """
        state = self.load_active_state()
        if state is None:
            if self.logger:
                self.logger.warning("No active state found to mark completed")
            return False

        state.status = DeploymentStatus.RUNNING.value
        state.error_message = ""

        if self.logger:
            self.logger.info(f"Active deployment completed: {state.stack_name} v{state.current_version}")

        return self.save_active_state(state)

    def mark_active_deployment_failed(self, error: str = "") -> bool:
        """
        Mark the active deployment as failed.

        Args:
            error: Error message describing the failure

        Returns:
            True if successful, False otherwise
        """
        state = self.load_active_state()
        if state is None:
            state = StackState(stack_name="_active")

        state.status = DeploymentStatus.FAILED.value
        state.error_message = error

        if self.logger:
            self.logger.error(f"Active deployment failed: {state.stack_name} - {error}")

        return self.save_active_state(state)

    def get_active_previous_stack(self) -> Optional[Dict[str, Any]]:
        """
        Get the previous stack from the global active state for rollback.

        This returns the last successfully running stack, regardless of its name.

        Returns:
            Previous stack definition if available, None otherwise
        """
        state = self.load_active_state()
        if state and state.previous_stack:
            prev_name = state.previous_stack.get("metadata", {}).get("name", "unknown")
            if self.logger:
                self.logger.info(f"Retrieved previous stack for rollback: {prev_name}")
            return state.previous_stack
        return None

    def can_rollback_active(self) -> bool:
        """
        Check if rollback is possible using the global active state.

        Returns:
            True if a previous stack exists for rollback
        """
        state = self.load_active_state()
        can_rollback = state is not None and state.previous_stack is not None
        if self.logger:
            if can_rollback:
                prev_name = state.previous_stack.get("metadata", {}).get("name", "unknown")
                self.logger.debug(f"Rollback available to: {prev_name}")
            else:
                self.logger.debug("No previous stack available for rollback")
        return can_rollback

    def mark_active_rollback_completed(self) -> bool:
        """
        Mark that a rollback was completed successfully in the active state.

        Swaps current and previous stacks to reflect the rollback.

        Returns:
            True if successful, False otherwise
        """
        state = self.load_active_state()
        if state is None:
            if self.logger:
                self.logger.warning("No active state found for rollback completion")
            return False

        # Swap current with previous (rollback)
        failed_stack = state.current_stack
        failed_version = state.current_version

        state.current_stack = state.previous_stack
        state.current_version = state.previous_version
        state.stack_name = self._get_stack_name_from_stack(state.current_stack) or "_active"
        state.stack_id = self._get_stack_id_from_stack(state.current_stack)
        state.previous_stack = failed_stack
        state.previous_version = failed_version
        state.status = DeploymentStatus.ROLLED_BACK.value
        state.rollback_count += 1

        if self.logger:
            self.logger.info(
                f"Active rollback completed: restored {state.stack_name} v{state.current_version}"
            )

        return self.save_active_state(state)
