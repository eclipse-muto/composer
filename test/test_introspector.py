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

import unittest
from unittest.mock import patch, call
import subprocess
from composer.introspection.introspector import Introspector


class TestIntrospector(unittest.TestCase):
    def setUp(self):
        self.introspector = Introspector()

    @patch("subprocess.run")
    @patch("builtins.print")
    def test_kill_success(self, mock_print, mock_run):
        mock_run.return_value = subprocess.CompletedProcess(
            args=["kill", "2025"], returncode=0, stdout="", stderr=""
        )

        self.introspector.kill("test_process", 2025)

        mock_run.assert_called_once_with(
            ["kill", "2025"], check=True, capture_output=True, text=True
        )

        expected_print_calls = [
            call("Attempting to kill test_process with PID: 2025"),
            call("Successfully killed test_process with PID: 2025"),
        ]
        mock_print.assert_has_calls(expected_print_calls)

    @patch("subprocess.run")
    @patch("builtins.print")
    def test_kill_called_process_error(self, mock_print, mock_run):
        mock_run.side_effect = subprocess.CalledProcessError(
            returncode=1, cmd=["kill", "2025"], stderr="Permission denied"
        )

        self.introspector.kill("test_process", 2025)

        mock_run.assert_called_once_with(
            ["kill", "2025"], check=True, capture_output=True, text=True
        )

        expected_print_calls = [
            call("Attempting to kill test_process with PID: 2025"),
            call("Kill was not successful for test_process. Error: Permission denied"),
        ]
        mock_print.assert_has_calls(expected_print_calls)

    @patch("subprocess.run")
    @patch("builtins.print")
    def test_kill_unexpected_error(self, mock_print, mock_run):
        mock_run.side_effect = PermissionError("Operation not permitted")

        self.introspector.kill("test_process", 2025)

        mock_run.assert_called_once_with(
            ["kill", "2025"], check=True, capture_output=True, text=True
        )

        expected_print_calls = [
            call("Attempting to kill test_process with PID: 2025"),
            call(
                "Unexpected error while trying to kill test_process. Exception message: Operation not permitted"
            ),
        ]
        mock_print.assert_has_calls(expected_print_calls)


if __name__ == "__main__":
    unittest.main()
