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

    def tearDown(self):
        pass

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



    @patch('multiprocessing.Process')
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
        self.assertEqual(mock_process.call_args[1]['daemon'], True)

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
        # mock_logger.get_logger().info.assert_called_with("Sent SIGKILL to process node1 (PID 1234)")
        mock_logger.get_logger().info.assert_called_with(
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


if __name__ == "__main__":

    unittest.main()
