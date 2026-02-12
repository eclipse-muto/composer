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
Muto path configuration module.

Provides centralized path configuration for Muto workspaces and state storage.
Uses MUTO_ROOT environment variable if set, otherwise defaults to ~/.muto.
"""

import os


def get_muto_root() -> str:
    """Returns the Muto root directory.

    Uses MUTO_ROOT environment variable if set, otherwise defaults to ~/.muto.
    The path is expanded to handle ~ properly.

    Returns:
        str: The absolute path to the Muto root directory.
    """
    return os.path.expanduser(os.environ.get("MUTO_ROOT", "~/.muto"))


def get_workspaces_path() -> str:
    """Returns the workspaces directory path.

    This is where extracted stack artifacts are stored and built.

    Returns:
        str: The absolute path to the workspaces directory.
    """
    return os.path.join(get_muto_root(), "workspaces")


def get_state_path() -> str:
    """Returns the state directory path.

    This is where persistent state files are stored for rollback support.

    Returns:
        str: The absolute path to the state directory.
    """
    return os.path.join(get_muto_root(), "state")


def ensure_directories() -> None:
    """Ensure all required Muto directories exist.

    Creates the root, workspaces, and state directories if they don't exist.
    """
    os.makedirs(get_muto_root(), exist_ok=True)
    os.makedirs(get_workspaces_path(), exist_ok=True)
    os.makedirs(get_state_path(), exist_ok=True)


# For backwards compatibility, export WORKSPACES_PATH as a module-level constant
# Note: This is evaluated at import time, so env var changes after import won't affect it
WORKSPACES_PATH = get_workspaces_path()
ARTIFACT_STATE_FILE = ".muto_artifact.json"
