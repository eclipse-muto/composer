#
#  Copyright (c) 2025 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#

import unittest
from unittest.mock import MagicMock, patch

import rclpy

from composer.workflow.router import Router


class TestRouter(unittest.TestCase):

    def setUp(self):
        self.pipelines = {
            "start": MagicMock(),
            "kill": MagicMock(),
            "apply": MagicMock(),
        }
        self.payload = {"stackId": "org.eclipse.muto.sandbox:test", "action": "kill"}
        self.pipeline = self.pipelines.get("kill")

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    @patch("rclpy.logging")
    @patch("composer.workflow.router.Pipeline.execute_pipeline")
    def test_route(self, mock_pipeline, mock_logging):
        main_route = Router(self.pipelines)
        main_route.route(self.payload.get("action", ""))
        self.pipeline.execute_pipeline.assert_called_once()
        mock_logging.get_logger().warn.assert_not_called()
        mock_pipeline.assert_not_called()

    @patch("rclpy.logging")
    def test_route_no_pipeline(self, mock_logging):
        self.pipelines = {"test": None}
        main_route = Router(self.pipelines)
        main_route.route(self.payload.get("action", ""))
        self.pipeline.execute_pipeline.assert_not_called()
        mock_logging.get_logger().warn.assert_called_once_with(
            "No pipeline found for action: kill"
        )


if __name__ == "__main__":
    unittest.main()
