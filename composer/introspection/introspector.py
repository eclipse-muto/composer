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

import subprocess

class Introspector():
    """
    A ROS 2 node for introspecting and managing processes.
    """
    def __init__(self):
        """Initialize the introspector."""

    def kill(self, name, pid):
        """
        Kills a process by its PID.

        Args:
            name: The name of the process to kill.
            pid: The process ID of the process to kill.
        """
        print(f'Attempting to kill {name} with PID: {pid}')
        try:
            result = subprocess.run(['kill', str(pid)], check=True, capture_output=True, text=True)
            if result.returncode == 0:
                print(f'Successfully killed {name} with PID: {pid}')
            else:
                print(f'Failed to kill {name} with PID: {pid}. Return code: {result.returncode}')
        except subprocess.CalledProcessError as e:
            print(f'Kill was not successful for {name}. Error: {e.stderr}')
        except Exception as e:
            print(f'Unexpected error while trying to kill {name}. Exception message: {e}')