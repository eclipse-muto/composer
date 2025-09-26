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
import rclpy
import json
import base64
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

    @patch.object(MutoComposer, "determine_execution_path")
    @patch.object(MutoComposer, "publish_raw_stack")
    @patch.object(MutoComposer, "publish_next_stack")
    @patch.object(MutoComposer, "resolve_expression")
    @patch("json.loads")
    def test_on_stack_callback_missing_key(self, mock_json_loads, mock_resolve_expression, 
                                         mock_publish_next_stack, mock_publish_raw_stack, 
                                         mock_determine_execution_path):
        # Test that missing 'value' key uses payload directly as stack
        mock_json_loads.return_value = {"not_value": {"stackId": "test_stack_id"}}
        mock_resolve_expression.return_value = '{"resolved": "stack"}'
        stack_msg = MagicMock()
        stack_msg.method = "start"
        stack_msg.payload = '{"not_value": {"stackId": "test_stack_id"}}'

        self.node.on_stack_callback(stack_msg)

        # Verify that it uses the payload directly as stack (else branch behavior)
        mock_resolve_expression.assert_called_once()
        mock_publish_next_stack.assert_called_once_with('{"resolved": "stack"}')
        mock_publish_raw_stack.assert_called_once_with('{"resolved": "stack"}')
        mock_determine_execution_path.assert_called_once()

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

    @patch("builtins.open")
    def test_load_pipeline_config_valid(self, mock_open):
        valid_config = {
            "pipelines": [
                {
                    "name": "test_pipeline",
                    "pipeline": ["action1", "action2"],
                    "compensation": ["undo1"],
                }
            ]
        }
        with patch("yaml.safe_load", return_value=valid_config), patch(
            "composer.muto_composer.validate"
        ) as mock_validate:
            config = self.node.load_pipeline_config("dummy_path")
            mock_validate.assert_called_once_with(
                instance={
                    "pipelines": [
                        {
                            "name": "test_pipeline",
                            "pipeline": ["action1", "action2"],
                            "compensation": ["undo1"],
                        }
                    ]
                },
                schema={
                    "type": "object",
                    "properties": {
                        "pipelines": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "name": {"type": "string"},
                                    "pipeline": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "sequence": {
                                                    "type": "array",
                                                    "items": {
                                                        "type": "object",
                                                        "properties": {
                                                            "name": {"type": "string"},
                                                            "service": {
                                                                "type": "string"
                                                            },
                                                            "plugin": {
                                                                "type": "string"
                                                            },
                                                            "condition": {
                                                                "type": "string"
                                                            },
                                                        },
                                                        "required": [
                                                            "name",
                                                            "service",
                                                            "plugin",
                                                        ],
                                                    },
                                                }
                                            },
                                            "required": ["sequence"],
                                        },
                                    },
                                    "compensation": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "service": {"type": "string"},
                                                "plugin": {"type": "string"},
                                            },
                                            "required": ["service", "plugin"],
                                        },
                                    },
                                },
                                "required": ["name", "pipeline", "compensation"],
                            },
                        }
                    },
                    "required": ["pipelines"],
                },
            )
            self.assertIn("pipelines", config)
            self.assertEqual(len(config["pipelines"]), 1)

    @patch.object(MutoComposer, "get_logger")
    def test_set_stack_done_callback(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        future = MagicMock()
        self.node.set_stack_done_callback(future)
        mock_logger.info.assert_called_with(
            "Edge device stack setting completed successfully."
        )

    @patch.object(MutoComposer, "get_logger")
    def test_set_stack_done_callback_exception(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        future = MagicMock()
        future.result = None
        self.node.set_stack_done_callback(future)
        mock_logger.error.assert_called_with(
            "Exception in set_stack_done_callback: 'NoneType' object is not callable"
        )

    def test_publish_current_stack(self):
        test_stack = '{"test": "stack"}'
        with patch.object(self.node.current_stack_publisher, "publish") as mock_pub:
            self.node.publish_current_stack(test_stack)
            mock_pub.assert_called_once()
            published_msg = mock_pub.call_args[0][0]
            self.assertEqual(published_msg.data, test_stack)

    def test_publish_next_stack(self):
        test_stack = '{"next": "stack"}'
        with patch.object(self.node.next_stack_publisher, "publish") as mock_pub:
            self.node.publish_next_stack(test_stack)
            mock_pub.assert_called_once()
            published_msg = mock_pub.call_args[0][0]
            self.assertEqual(published_msg.data, test_stack)

    def test_determine_execution_path_empty_next_stack(self):
        self.node.current_stack = {"launch_description_source": "existing"}
        self.node.next_stack = json.dumps({"launch_description_source": "new_launch"})
        self.node.pipeline_execute = MagicMock()

        self.node.determine_execution_path()

        self.node.pipeline_execute.assert_called_once_with(
            self.node.method,
            {"should_run_provision": True, "should_run_launch": True},
        )

    def test_extract_stack_from_solution(self):
        stack_payload = {"name": "decoded", "artifact": {}}
        encoded = base64.b64encode(json.dumps(stack_payload).encode("utf-8")).decode("ascii")
        solution_payload = {
            "metadata": {},
            "spec": {
                "components": [
                    {
                        "properties": {
                            "type": "stack",
                            "data": encoded,
                        }
                    }
                ]
            }
        }

        decoded = self.node.stack_parser.parse_payload(solution_payload)
        self.assertEqual(decoded, stack_payload)

    def test_parse_payload_non_dict(self):
        """Test that parse_payload returns None for non-dict payloads"""
        result = self.node.stack_parser.parse_payload("not a dict")
        self.assertIsNone(result)

    def test_parse_payload_with_value_key(self):
        """Test that parse_payload returns payload as-is when it has a 'value' key"""
        payload = {"value": {"stackId": "test-stack"}}
        result = self.node.stack_parser.parse_payload(payload)
        self.assertEqual(result, payload)

    def test_parse_payload_direct_stack_json(self):
        """Test parsing direct stack JSON format"""
        payload = {
            "metadata": {
                "name": "Test Stack",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [
                    {
                        "name": "test_node",
                        "pkg": "test_pkg",
                        "exec": "test_exec"
                    }
                ]
            }
        }
        result = self.node.stack_parser.parse_payload(payload)
        expected = {
            "node": [
                {
                    "name": "test_node",
                    "pkg": "test_pkg",
                    "exec": "test_exec"
                }
            ]
        }
        self.assertEqual(result, expected)

    def test_parse_payload_archive_format(self):
        """Test parsing archive format"""
        payload = {
            "metadata": {
                "name": "Test Archive Stack",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "dGVzdCBkYXRh",  # base64 encoded "test data"
                "properties": {
                    "launch_file": "launch/test.launch.py",
                    "command": "launch",
                    "launch_args": [
                        {"name": "arg1", "default": "val1"}
                    ]
                }
            }
        }
        result = self.node.stack_parser.parse_payload(payload)
        self.assertIsNotNone(result)
        self.assertEqual(result["content_type"], "stack/archive")
        self.assertEqual(result["stack"], "dGVzdCBkYXRh")
        self.assertEqual(result["launch_file"], "launch/test.launch.py")
        self.assertEqual(result["command"], "launch")
        self.assertEqual(result["launch_args"], [{"name": "arg1", "default": "val1"}])

    def test_parse_payload_unparseable(self):
        """Test that parse_payload returns None for unparseable payloads"""
        payload = {
            "unknown": "format",
            "no": "matching keys"
        }
        result = self.node.stack_parser.parse_payload(payload)
        self.assertIsNone(result)

    def test_parse_payload_direct_stack_json_string_launch(self):
        """Test parsing direct stack JSON format with string launch data"""
        payload = {
            "metadata": {
                "content_type": "stack/json"
            },
            "launch": '{"node": [{"name": "string_node", "pkg": "string_pkg"}]}'
        }
        result = self.node.stack_parser.parse_payload(payload)
        expected = {"node": [{"name": "string_node", "pkg": "string_pkg"}]}
        self.assertEqual(result, expected)

    def test_parse_payload_invalid_direct_stack_json(self):
        """Test parsing invalid direct stack JSON format"""
        payload = {
            "metadata": {
                "content_type": "stack/json"
            },
            "launch": "invalid json string {{{"
        }
        result = self.node.stack_parser.parse_payload(payload)
        self.assertIsNone(result)

    def test_determine_execution_path_with_artifact(self):
        artifact_stack = {
            "metadata": {
                "name": "test-artifact",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "ZHVtbXk=",
                "properties": {
                    "filename": "dummy.tar.gz"
                }
            }
        }
        self.node.current_stack = {}
        self.node.next_stack = json.dumps(artifact_stack)
        self.node.pipeline_execute = MagicMock()

        self.node.determine_execution_path()

        self.node.pipeline_execute.assert_called_once_with(
            self.node.method,
            {"should_run_provision": True, "should_run_launch": True},
        )

    @patch("composer.muto_composer.Stack")
    def test_merge(self, mock_stack):
        stack1 = {"node": ["node1"], "composable": ["comp1"]}
        stack2 = {"node": ["node2"], "args": {"arg1": "value1"}}

        merged = self.node.merge(stack1, stack2)

        self.assertEqual(mock_stack.call_count, 2)

        merged = self.node.merge(None, stack2)
        self.assertEqual(merged, stack2)

    def test_pipeline_execute_valid(self):
        test_pipeline = MagicMock()
        self.node.pipelines = {"test_pipeline": test_pipeline}
        self.node.pipeline_execute("test_pipeline", {"key": "value"})
        test_pipeline.execute_pipeline.assert_called_once_with(
            additional_context={"key": "value"}
        )

    @patch.object(MutoComposer, "get_logger")
    def test_pipeline_execute_invalid(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger
        self.node.pipelines = {}
        self.node.pipeline_execute("invalid_pipeline")
        mock_logger.warn.assert_called_with(
            "No pipeline found with name: invalid_pipeline"
        )


if __name__ == "__main__":
    unittest.main()
