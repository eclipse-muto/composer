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
from unittest.mock import MagicMock, patch, AsyncMock
import rclpy
from composer.workflow.launcher import Ros2LaunchParent


class TestLauncherAsync(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.launch_arguments = MagicMock()

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    @patch("rclpy.logging")
    @patch("composer.workflow.launcher.OnProcessExit")
    @patch("composer.workflow.launcher.RegisterEventHandler")
    @patch.object(Ros2LaunchParent, "parse_launch_arguments")
    @patch("composer.workflow.launcher.launch")
    async def test_launch_a_launch_file_test(
        self,
        mock_launch,
        mock_parse_launch_arguments,
        mock_register_event_handler,
        mock_on_process_exit,
        mock_logger,
    ):
        mock_ls_instance = MagicMock()
        mock_ls_instance.run_async = AsyncMock()
        mock_launch.LaunchService.return_value = mock_ls_instance

        launch_file_path = "/test/path"
        launch_file_arguments = MagicMock()
        noninteractive = False
        debug = False
        dry_run = False

        parent = Ros2LaunchParent(self.launch_arguments)

        await parent.launch_a_launch_file(
            launch_file_path, launch_file_arguments, noninteractive, debug, dry_run
        )

        mock_parse_launch_arguments.assert_called_once_with(launch_file_arguments)
        mock_ls_instance.run_async.assert_awaited_once()
        mock_launch.LaunchDescription.assert_called_once_with(
            [mock_launch.actions.IncludeLaunchDescription()]
        )
        mock_launch.LaunchDescription().add_action.assert_called_with(
            mock_register_event_handler()
        )
        self.assertEqual(mock_launch.LaunchDescription().add_action.call_count, 2)
        mock_logger.get_logger().info.assert_called_once()
        mock_on_process_exit.assert_called_once()


if __name__ == "__main__":
    unittest.main()
