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

import signal
import unittest
from unittest.mock import MagicMock, call, patch
from launch import LaunchDescription

import rclpy

from composer.workflow.launcher import Ros2LaunchParent


class TestLauncher(unittest.TestCase):

    def setUp(self):
        self.manager = MagicMock()
        self._active_nodes = self.manager.list()
        self._lock = self.manager.Lock()
        self._stop_event = MagicMock()
        self._process = MagicMock()
        self._run_process = MagicMock()
        self.launch_arguments = MagicMock()

        self.launch_args = []
        self.launch_parent = Ros2LaunchParent(self.launch_args)

        self.logger_mock = MagicMock()
        rclpy.logging.get_logger = MagicMock(return_value=self.logger_mock)

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_del_(self):
        Ros2LaunchParent.__del__(self)
        self.manager.shutdown.assert_called_once()

    def test_parse_launch_arguments(self):
        launch_list = ["launch_css:=false", "launch_sensor:=false"]
        returned_value = Ros2LaunchParent.parse_launch_arguments(
            self, launch_arguments=launch_list
        )
        self.assertIn(("launch_css", "false"), returned_value)
        self.assertIn(("launch_sensor", "false"), returned_value)

    def test_runtime_error_parse_launch_argument(self):
        launch_list = ["launch:1"]
        with self.assertRaises(RuntimeError):
            Ros2LaunchParent.parse_launch_arguments(self, launch_arguments=launch_list)

    def test_multi_parse_launch_argument(self):
        launch_list = [
            "launch_css:=false",
            "launch_sensor:=false",
            "launch_sensor:=true",
        ]
        returned_value = Ros2LaunchParent.parse_launch_arguments(
            self, launch_arguments=launch_list
        )
        self.assertIn(("launch_css", "false"), returned_value)
        self.assertIn(("launch_sensor", "true"), returned_value)
        self.assertNotIn(("la0unch_sensor", "false"), returned_value)

    @patch("multiprocessing.Process")
    def test_start_method(self, mock_process):
        ld = LaunchDescription()
        parent = Ros2LaunchParent(ld)
        process_mock = MagicMock()
        mock_process.return_value = process_mock

        parent.start(ld)

        mock_process.assert_called_once()
        process_mock.start.assert_called_once()
        self.assertIsNotNone(parent._stop_event)
        self.assertIsNotNone(parent._process)
        self.assertEqual(mock_process.call_args[1]["daemon"], True)

    @patch("rclpy.logging")
    def test_shutdown_is_alive_true(self, mock_logger):
        Ros2LaunchParent.shutdown(self)
        self._stop_event.set.assert_called_once()
        self._process.join.assert_called_with(timeout=20.0)
        self._process.is_alive.return_value = True
        self._process.terminate.assert_called_once()
        mock_logger.get_logger().info.assert_called_once_with(
            "The process did not terminate gracefully and was terminated forcefully."
        )

    def test_shutdown_is_alive_false(self):
        self._process.is_alive.return_value = False
        Ros2LaunchParent.shutdown(self)
        self._stop_event.set.assert_called_once()
        self._process.join.assert_called_with(timeout=20.0)
        self._process.terminate.assert_not_called()

    @patch("rclpy.logging")
    @patch("os.kill")
    def test_kill_no_active_nodes(self, mock_kill, mock_logger):
        launch_arguments = {}
        ros2launch_parent = Ros2LaunchParent(launch_arguments)
        ros2launch_parent._active_nodes = []
        ros2launch_parent._lock = MagicMock()

        ros2launch_parent.kill()
        ros2launch_parent._lock.__enter__.assert_called_once()
        mock_kill.assert_not_called()
        mock_logger.get_logger().info.assert_called_once_with(
            "No active nodes to kill."
        )

    @patch("rclpy.logging")
    @patch("os.kill")
    def test_kill_active_nodes(self, mock_kill, mock_logger):
        launch_arguments = {}
        ros2launch_parent = Ros2LaunchParent(launch_arguments)
        ros2launch_parent._active_nodes = [{"node1": 1234}, {"node2": 5678}]
        ros2launch_parent._lock = MagicMock()

        ros2launch_parent.kill()
        ros2launch_parent._lock.__enter__.assert_called_once()
        mock_kill.assert_has_calls(
            [call(1234, signal.SIGKILL), call(5678, signal.SIGKILL)], any_order=True
        )
        self.assertEqual(ros2launch_parent._active_nodes, [])
        # Check that both nodes were logged (order may vary due to parallel execution)
        mock_logger.get_logger().info.assert_any_call(
            "Sent SIGKILL to process node1 (PID 1234)"
        )
        mock_logger.get_logger().info.assert_any_call(
            "Sent SIGKILL to process node2 (PID 5678)"
        )

    @patch("rclpy.logging")
    @patch("os.kill", side_effect=ProcessLookupError)
    def test_kill_process_already_terminated(self, mock_kill, mock_logger):
        launch_arguments = {}
        ros2launch_parent = Ros2LaunchParent(launch_arguments)
        ros2launch_parent._active_nodes = [{"node1": 1234}]
        ros2launch_parent._lock = MagicMock()

        ros2launch_parent.kill()
        ros2launch_parent._lock.__enter__.assert_called_once()
        mock_kill.assert_called_once_with(1234, signal.SIGKILL)
        mock_logger.get_logger().info.assert_called_once_with(
            "Process node1 (PID 1234) already terminated."
        )

    @patch("rclpy.logging")
    @patch("os.kill", side_effect=Exception("Some error"))
    def test_kill_exception(self, mock_kill, mock_logger):
        launch_arguments = {}
        ros2launch_parent = Ros2LaunchParent(launch_arguments)
        ros2launch_parent._active_nodes = [{"node1": 1234}]
        ros2launch_parent._lock = MagicMock()

        ros2launch_parent.kill()
        ros2launch_parent._lock.__enter__.assert_called_once()
        mock_kill.assert_called_once_with(1234, signal.SIGKILL)
        self.assertEqual(ros2launch_parent._active_nodes, [])
        mock_logger.get_logger().info.assert_called_once_with(
            "Failed to kill process node1 (PID 1234): Some error"
        )

    @patch("rclpy.logging")
    @patch("os.kill")
    def test_kill_process(self, mock_kill, mock_logger):
        launch_arguments = {}
        ros2launch_parent = Ros2LaunchParent(launch_arguments)
        ros2launch_parent._active_nodes = [{"node1": 1234}]
        ros2launch_parent._lock = MagicMock()
        mock_process = MagicMock()
        mock_process.is_alive.return_value = False
        ros2launch_parent._process = mock_process
        ros2launch_parent._stop_event = MagicMock()

        ros2launch_parent.kill()
        mock_process.join.assert_called_once_with(timeout=10.0)
        ros2launch_parent._stop_event.set.assert_called_once()
        self.assertEqual(ros2launch_parent._active_nodes, [])
        mock_logger.get_logger().info.assert_called_with(
            "Shutting down the launch service"
        )
        mock_kill.assert_called_once()

    @patch("rclpy.logging")
    @patch("os.kill")
    def test_kill_alive_process(self, mock_kill, mock_logger):
        launch_arguments = {}
        ros2launch_parent = Ros2LaunchParent(launch_arguments)
        ros2launch_parent._active_nodes = [{"node1": 1234}]
        ros2launch_parent._lock = MagicMock()
        mock_process = MagicMock()
        mock_process.is_alive.return_value = True
        ros2launch_parent._process = mock_process
        ros2launch_parent._stop_event = MagicMock()

        ros2launch_parent.kill()
        mock_process.terminate.assert_called_once()
        mock_process.join.assert_called_once_with(timeout=10.0)
        self.assertEqual(ros2launch_parent._active_nodes, [])
        mock_logger.get_logger().info.assert_called_with(
            "Shutting down the launch service"
        )
        mock_kill.assert_called_once()

    @patch("rclpy.logging")
    def test_event_handler_start(self, mock_logger):
        mock_action_start = MagicMock()
        Ros2LaunchParent._event_handler(
            self, "start", mock_action_start, self._active_nodes, self._lock
        )
        self._active_nodes.append.assert_called_once_with(
            {mock_action_start.process_name: mock_action_start.pid}
        )
        mock_logger.get_logger().info.assert_called()

    @patch("rclpy.logging")
    def test_event_handler_exit(self, mock_logger):
        mock_action_exit = MagicMock()
        mock_action_exit.process_name = "test_node"
        mock_action_exit.pid = 24
        node_list = [{"test_node": 24}]
        Ros2LaunchParent._event_handler(
            MagicMock(), "exit", mock_action_exit, node_list, self._lock
        )
        self.assertNotIn({"test_node": 24}, node_list)
        mock_logger.get_logger().info.assert_called_with("Active nodes after exit: []")

    @patch("asyncio.new_event_loop")
    @patch("asyncio.set_event_loop")
    def test_run_process(self, mock_set, mock_new):
        ld = LaunchDescription()
        node = Ros2LaunchParent(ld)
        mock_stop_event = MagicMock()
        node._run_process(mock_stop_event, ld)
        mock_new.assert_called_once()
        mock_set.assert_called_once()

    @patch("composer.workflow.launcher.Node")
    @patch("launch.LaunchDescription.add_action")
    def test_create_launch_description_for_added_nodes(
        self, mock_add_action, mock_node
    ):
        added_nodes = {
            "node1": {
                "name": "test_node",
                "namespace": "/test_ns",
                "package": "test_pkg",
                "executable": "test_exe",
                "parameters": [{"param1": "value1"}],
            }
        }
        ld = MagicMock()
        node = Ros2LaunchParent(ld)

        node.create_launch_description_for_added_nodes(added_nodes)
        call_value = mock_node(
            package="test_pkg",
            executable="test_exe",
            name="test_node",
            namespace="/test_ns",
            output="screen",
        )
        mock_add_action.assert_called_once_with(call_value)

    @patch("os.kill")
    def test_kill_single_node(self, mock_kill):
        with self.launch_parent._lock:
            self.launch_parent._active_nodes.append({"node1": 1234})
            self.launch_parent._active_nodes.append({"node2": 5678})

        self.launch_parent.kill_nodes_by_name(["node1"])

        mock_kill.assert_called_once_with(1234, signal.SIGKILL)

        with self.launch_parent._lock:
            self.assertEqual(len(self.launch_parent._active_nodes), 1)
            self.assertEqual(self.launch_parent._active_nodes[0], {"node2": 5678})

        self.logger_mock.info.assert_any_call(
            "Sent SIGKILL to process node1 (PID 1234)"
        )

    @patch("os.kill")
    def test_kill_multiple_nodes(self, mock_kill):
        with self.launch_parent._lock:
            self.launch_parent._active_nodes.append({"node1": 1234})
            self.launch_parent._active_nodes.append({"node2": 5678})
            self.launch_parent._active_nodes.append({"node3": 9101})

        self.launch_parent.kill_nodes_by_name(["node1", "node3"])

        self.assertEqual(mock_kill.call_count, 2)
        mock_kill.assert_any_call(1234, signal.SIGKILL)
        mock_kill.assert_any_call(9101, signal.SIGKILL)

        with self.launch_parent._lock:
            self.assertEqual(len(self.launch_parent._active_nodes), 1)
            self.assertEqual(self.launch_parent._active_nodes[0], {"node2": 5678})

    @patch("os.kill", side_effect=ProcessLookupError)
    def test_kill_already_terminated_node(self, mock_kill):
        with self.launch_parent._lock:
            self.launch_parent._active_nodes.append({"node1": 1234})

        self.launch_parent.kill_nodes_by_name(["node1"])

        mock_kill.assert_called_once_with(1234, signal.SIGKILL)

        with self.launch_parent._lock:
            self.assertEqual(len(self.launch_parent._active_nodes), 0)

        self.logger_mock.info.assert_any_call(
            "Process node1 (PID 1234) already terminated."
        )

    @patch("os.kill", side_effect=PermissionError("No permission"))
    def test_kill_node_permission_error(self, mock_kill):
        with self.launch_parent._lock:
            self.launch_parent._active_nodes.append({"node1": 1234})

        self.launch_parent.kill_nodes_by_name(["node1"])

        mock_kill.assert_called_once_with(1234, signal.SIGKILL)

        with self.launch_parent._lock:
            self.assertEqual(len(self.launch_parent._active_nodes), 0)

        self.logger_mock.error.assert_called_once_with(
            "Failed to kill process node1 (PID 1234): No permission"
        )

    @patch("os.kill")
    def test_kill_nodes_with_no_active_nodes(self, mock_kill):
        with self.launch_parent._lock:
            self.assertEqual(len(self.launch_parent._active_nodes), 0)

        self.launch_parent.kill_nodes_by_name(["node1"])

        mock_kill.assert_not_called()

        self.logger_mock.info.assert_any_call("No active nodes to kill.")


if __name__ == "__main__":

    unittest.main()
