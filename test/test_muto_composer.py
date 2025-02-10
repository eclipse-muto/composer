import unittest
import rclpy
import json
from composer.muto_composer import MutoComposer
from unittest.mock import MagicMock, patch
from muto_msgs.srv import CoreTwin
from std_msgs.msg import String
from rclpy.task import Future
from muto_msgs.msg import MutoAction


class TestMutoComposer(unittest.TestCase):
    @patch("composer.muto_composer.MutoComposer.init_pipelines")
    @patch("composer.muto_composer.Pipeline")
    @patch("composer.muto_composer.MutoComposer.bootstrap")
    def setUp(self, mock_bootstrap, mock_pipeline, mock_pipe) -> None:
        self.node = MutoComposer()
        self.incoming_stack_topic = MagicMock()
        self.get_stack_cli = MagicMock()
        self.incoming_stack = None
        self.method = None
        self.raw_stack_publisher = MagicMock()
        self.pipeline_file_path = "/composer/config/config.yaml"
        self.router = MagicMock()
        self.node.pipelines = MagicMock()

        self.logger = MagicMock()
        self.get_logger = MagicMock()

    def tearDown(self) -> None:
        self.node.destroy_node()

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    @patch("json.loads")
    def test_on_stack_callback(self, mock_json):
        stack_msg = MagicMock()
        self.node.get_logger = MagicMock()
        self.node.get_stack_cli = MagicMock()
        self.node.get_stack_cli.call_async = MagicMock()

        stack_msg.method = "start"
        stack_msg.payload = json.dumps({"value": {"stackId": "8"}})
        mock_json.return_value = {"value": {"stackId": "8"}}
        self.node.on_stack_callback(stack_msg)
        self.assertEqual(self.node.method, "start")
        self.node.get_stack_cli.call_async.assert_called_once()
        async_value = self.node.get_stack_cli.call_async.return_value
        async_value.add_done_callback.assert_called_once_with(
            self.node.get_stack_done_callback
        )

    @patch.object(MutoComposer, "get_logger")
    @patch("json.loads")
    def test_on_stack_callback_general_exception(
        self, mock_json_loads, mock_get_logger
    ):
        mock_json_loads.side_effect = Exception("General error")
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        stack_msg = MagicMock()
        stack_msg.payload = "{}"

        self.node.on_stack_callback(stack_msg)

        mock_logger.error.assert_called_with(
            "Error parsing stack from agent: General error"
        )

    @patch.object(MutoComposer, "get_logger")
    def test_on_stack_callback_invalid_json(self, mock_get_logger):
        stack_msg = MagicMock()
        stack_msg.payload = "Invalid JSON"
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        self.node.on_stack_callback(stack_msg)

        mock_logger.error.assert_called_with(
            "Invalid JSON in payload: Expecting value: line 1 column 1 (char 0)"
        )

    @patch.object(MutoComposer, "get_logger")
    @patch("json.loads")
    def test_on_stack_callback_missing_key(self, mock_json_loads, mock_get_logger):
        mock_json_loads.return_value = {"not_value": {"stackId": "test_stack_id"}}
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        stack_msg = MagicMock()
        stack_msg.payload = '{"not_value": {"stackId": "test_stack_id"}}'

        self.node.on_stack_callback(stack_msg)

        mock_logger.error.assert_called_with("Payload is missing key: 'value'")

    @patch.object(MutoComposer, "resolve_expression")
    @patch.object(MutoComposer, "determine_execution_path")
    @patch.object(MutoComposer, "publish_raw_stack")
    @patch.object(MutoComposer, "publish_next_stack")
    @patch("composer.muto_composer.Future")
    def test_get_stack_done_callback(
        self,
        mock_future,
        mock_pb_next_stack,
        mock_pb_raw_stack,
        mock_determine_execution_path,
        mock_resolve_expression,
    ):
        mock_future.result().return_value = MagicMock()
        mock_future.result().output = json.dumps({"output": "test_out"})
        self.node.get_stack_done_callback(mock_future)

        mock_pb_next_stack.assert_called_once_with(
            mock_resolve_expression(mock_future.result().output)
        )
        mock_pb_raw_stack.assert_called_once_with(
            mock_resolve_expression(mock_future.result().output)
        )
        mock_determine_execution_path.assert_called_once_with()

    @patch.object(MutoComposer, "pipeline_execute")
    @patch.object(MutoComposer, "publish_raw_stack")
    @patch.object(MutoComposer, "publish_current_stack")
    @patch("composer.muto_composer.MutoComposer.resolve_expression")
    @patch("composer.muto_composer.Future")
    def test_activate(
        self,
        mock_future,
        mock_resolve_expression,
        mock_pb_current_stack,
        mock_pb_raw_stack,
        mock_pipeline_execute,
    ):
        mock_future.result().return_value = MagicMock()
        mock_future.result().output = json.dumps({"output": "test_out"})
        self.node.activate(future=mock_future)
        mock_resolve_expression.assert_called_once_with('{"output": "test_out"}')
        mock_pb_current_stack.assert_called_once()
        mock_pb_raw_stack.assert_called_once()
        mock_pipeline_execute.assert_called_once_with("start")

    @patch.object(MutoComposer, "get_logger")
    def test_activate_no_default_stack(self, mock_get_logger):
        self.node.bootstrap_pub = MagicMock()
        future = MagicMock(spec=Future)
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        future.result.side_effect = AttributeError()

        self.node.activate(future)

        self.node.bootstrap_pub.publish.assert_not_called()

        mock_logger.error.assert_any_call("No default stack. Aborting bootstrap")

    @patch.object(MutoComposer, "get_logger")
    def test_activate_generic_exception(self, mock_get_logger):
        self.node.bootstrap_pub = MagicMock()
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        future = MagicMock(spec=Future)
        future.result.side_effect = Exception("Unexpected error")

        self.node.activate(future)

        self.node.bootstrap_pub.publish.assert_not_called()

        mock_logger.error.assert_any_call("Error while bootstrapping: Unexpected error")

    @patch.object(MutoComposer, "get_logger")
    @patch("requests.get")
    def test_bootstrap_success(self, mock_requests_get, mock_get_logger):
        self.node.get_stack_cli = MagicMock()
        mock_response = MagicMock()
        mock_response.json.return_value = {"stackId": "my-stack-id"}
        mock_requests_get.return_value = mock_response
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        future_mock = MagicMock()
        self.node.get_stack_cli.call_async.return_value = future_mock

        self.node.bootstrap()

        expected_url = f"{self.node.twin_url}/api/2/things/{self.node.thing_id}/features/stack/properties/current"
        mock_requests_get.assert_called_once_with(
            expected_url,
            headers={"Content-type": "application/json"},
        )

        self.node.get_stack_cli.call_async.assert_called_once()
        args = self.node.get_stack_cli.call_async.call_args
        req = args[0][0]
        self.assertIsInstance(req, CoreTwin.Request)
        self.assertEqual(req.input, "my-stack-id")

        future_mock.add_done_callback.assert_called_once_with(self.node.activate)

    @patch.object(MutoComposer, "get_logger")
    def test_bootstrap_no_default_stack(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        self.node.get_stack_cli = MagicMock()
        with patch("requests.get", side_effect=AttributeError("No default stack")):
            self.node.bootstrap()

        self.node.get_stack_cli.call_async.assert_not_called()

        mock_logger.error.assert_called_once_with(
            "No default stack. Aborting bootstrap"
        )

    @patch("composer.muto_composer.get_package_share_directory")
    def test_resolve_expression_find(self, mock_get_package):
        mock_get_package.return_value = "/mock_path/test_pkg"
        resolve_expression_input = "$(find test_pkg)"
        self.node.resolve_expression(resolve_expression_input)
        mock_get_package.assert_called()

    @patch("composer.muto_composer.os.getenv")
    def test_resolve_expression_env(self, mock_get_env):
        mock_get_env.return_value = "test_env"
        resolve_expression_input = "$(env test_env)"
        self.node.resolve_expression(resolve_expression_input)
        mock_get_env.assert_called()

    @patch.object(MutoComposer, "get_logger")
    def test_resolve_expression_no_expression(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        resolve_expression_input = "$(test_exp test_pkg)"
        self.node.resolve_expression(resolve_expression_input)
        mock_logger.info.assert_called_with(
            "No muto expression found in the given string"
        )

    @patch.object(MutoComposer, "get_logger")
    def test_resolve_expression_key_error(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        input_value = "$(find demo_pkg)"
        with patch(
            "composer.muto_composer.get_package_share_directory", side_effect=KeyError
        ):
            result = self.node.resolve_expression(input_value)
        mock_logger.warn.assert_called_with("demo_pkg does not exist.")
        self.assertEqual(result, input_value)

    @patch.object(MutoComposer, "get_logger")
    def test_resolve_expression_exception(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        input_value = "$(find demo_pkg)"

        with patch(
            "composer.muto_composer.get_package_share_directory", side_effect=Exception
        ):
            result = self.node.resolve_expression(input_value)

        mock_logger.info.assert_called_with("Exception occurred: ")
        self.assertEqual(result, input_value)

    def test_publish_raw_stack(self):
        stack = "test_stack"
        expected_value = String(data=stack)
        MutoComposer.publish_raw_stack(self, stack)
        self.raw_stack_publisher.publish.assert_called_once_with(expected_value)

    @patch("composer.muto_composer.Pipeline")
    def test_init_pipelines(self, mock_pipeline):
        pipeline_config = [
            {
                "name": "test_name",
                "pipeline": "test_pipeline",
                "compensation": "test_compensation",
            }
        ]

        self.node.init_pipelines(pipeline_config)

        mock_pipeline.assert_called_once_with(
            "test_name", "test_pipeline", "test_compensation"
        )


if __name__ == "__main__":
    unittest.main()
