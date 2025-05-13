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
import composer.model.param as param
from unittest.mock import MagicMock, patch


class TestParam(unittest.TestCase):

    @patch("composer.model.param.Param._resolve_value")
    def setUp(self, mock_resolve_value):
        self.mock_stack = MagicMock()
        self.param = param.Param(
            stack=self.mock_stack,
            manifest={"name": "test-333", "from": "test", "command": "start"},
        )

    def test_parse_value(self):
        returned1 = self.param._parse_value("TRUE")
        returned2 = self.param._parse_value("FALSE")
        returned3 = self.param._parse_value("1")
        returned4 = self.param._parse_value("one")
        self.assertTrue(returned1)
        self.assertFalse(returned2)
        self.assertEqual(returned3, 1)
        self.assertEqual(returned4, "one")

    def test_execute_command(self):
        value = self.param._execute_command(command="echo muto")
        self.assertEqual(value, "muto")


if __name__ == "__main__":
    unittest.main()
