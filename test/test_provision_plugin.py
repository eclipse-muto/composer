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

import json
import os
import subprocess
import unittest
from unittest.mock import MagicMock, call, patch

import rclpy

from composer.plugins.provision_plugin import WORKSPACES_PATH, MutoProvisionPlugin


class TestMutoProvisionPlugin(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MutoProvisionPlugin()
        self.node.get_logger = MagicMock()
        # Don't set current_stack here - let handle_provision parse it from the request

    def _create_stack_json(self, content_type="stack/json", name="Test Stack", **kwargs):
        """Helper to create proper stack JSON for tests."""
        stack = {
            "metadata": {
                "name": name,
                "content_type": content_type
            }
        }
        stack.update(kwargs)
        return json.dumps(stack)


    def tearDown(self):
        self.node.destroy_node()



