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
from unittest.mock import patch, MagicMock
import rclpy
from rclpy.node import Node

from composer.workflow.pipeline import Pipeline


class TestPipeline(unittest.TestCase):
    def setUp(self):
        self.steps_config = [
            {
                "sequence": [
                    {
                        "name": "compose_step",
                        "service": "muto_compose",
                        "plugin": "ComposePlugin",
                    },
                    {
                        "name": "provision_step",
                        "service": "muto_provision",
                        "plugin": "ProvisionPlugin",
                        "condition": "should_run_provision == True",
                    },
                    {
                        "name": "launch_step",
                        "service": "muto_launch_stack",
                        "plugin": "LaunchPlugin",
                        "condition": "should_run_launch == True",
                    },
                ]
            }
        ]

        self.compensation_config = [
            {
                "name": "kill_step",
                "service": "muto_kill_stack",
                "plugin": "LaunchPlugin",
            }
        ]

    @patch("importlib.import_module")
    def test_load_plugins_success(self, mock_import_module):

        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            ProvisionPlugin=mock_plugin_class,
            LaunchPlugin=mock_plugin_class,
        )

        pipeline = Pipeline(
            name="test_pipeline",
            steps=self.steps_config,
            compensation=self.compensation_config,
        )

        self.assertIn("ComposePlugin", pipeline.plugins)
        self.assertIn("ProvisionPlugin", pipeline.plugins)
        self.assertIn("LaunchPlugin", pipeline.plugins)

    def test_load_plugins_failure(self):

        steps_config = [
            {
                "sequence": [
                    {
                        "name": "random_step",
                        "plugin": "RandomPlugin",
                        "service": "random_provision",
                    }
                ]
            }
        ]
        compensation_config = []

        with self.assertRaises(AttributeError):
            Pipeline(
                name="test_pipeline",
                steps=steps_config,
                compensation=compensation_config,
            )

    @patch("importlib.import_module")
    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_execute_pipeline_with_conditions_skipped_step(
        self, mock_create_node, mock_spin, mock_import_module
    ):

        mock_spin.return_value = None

        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            ProvisionPlugin=mock_plugin_class,
            LaunchPlugin=mock_plugin_class,
        )

        def create_client_side_effect(plugin, service_name):
            client_mock = MagicMock()
            client_mock.wait_for_service.return_value = True

            future_mock = MagicMock()
            future_mock.result.return_value = MagicMock(success=True, err_msg="")
            client_mock.call_async.return_value = future_mock

            return client_mock

        mock_node = MagicMock(spec=Node)
        mock_create_node.return_value = mock_node
        mock_node.create_client.side_effect = create_client_side_effect

        pipeline = Pipeline(
            name="test_pipeline",
            steps=self.steps_config,
            compensation=self.compensation_config,
        )

        context = {"should_run_provision": False, "should_run_launch": False}
        pipeline.execute_pipeline(additional_context=context)

        self.assertIn(
            "compose_step",
            pipeline.context,
            "compose_step should be recorded in pipeline.context",
        )
        self.assertNotIn(
            "provision_step",
            pipeline.context,
            "provision_step should not run, so not in pipeline.context",
        )
        self.assertNotIn(
            "launch_step",
            pipeline.context,
            "launch_step should not run, so not in pipeline.context",
        )

    @patch("importlib.import_module")
    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_execute_pipeline_failure_triggers_compensation(
        self, mock_create_node, mock_spin, mock_import_module
    ):
        mock_spin.return_value = None
        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            ProvisionPlugin=mock_plugin_class,
            LaunchPlugin=mock_plugin_class,
        )

        def create_client_side_effect(plugin, service_name):
            client_mock = MagicMock()
            client_mock.wait_for_service.return_value = True

            future_mock = MagicMock()

            if service_name == "muto_compose":
                future_mock.result.return_value = MagicMock(success=True, err_msg="")
            elif service_name == "muto_provision":
                future_mock.result.return_value = MagicMock(
                    success=False, err_msg="Provision error"
                )
            elif service_name == "muto_kill_stack":
                future_mock.result.return_value = MagicMock(success=True, err_msg="")
            else:
                future_mock.result.return_value = MagicMock(
                    success=True, err_msg="Unreached"
                )

            client_mock.call_async.return_value = future_mock
            return client_mock

        mock_node = MagicMock(spec=Node)
        mock_create_node.return_value = mock_node
        mock_node.create_client.side_effect = create_client_side_effect

        pipeline = Pipeline(
            name="test_pipeline",
            steps=self.steps_config,
            compensation=self.compensation_config,
        )

        context = {"should_run_provision": True, "should_run_launch": True}
        pipeline.execute_pipeline(additional_context=context)

        self.assertIn("compose_step", pipeline.context)
        self.assertTrue(pipeline.context["compose_step"].success)

        self.assertIn("provision_step", pipeline.context)
        self.assertFalse(pipeline.context["provision_step"].success)

        self.assertNotIn("launch_step", pipeline.context)

    @patch("importlib.import_module")
    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_execute_pipeline_condition_evaluation_error(
        self, mock_create_node, mock_spin, mock_import_module
    ):
        mock_spin.return_value = None
        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            ProvisionPlugin=mock_plugin_class,
            LaunchPlugin=mock_plugin_class,
        )

        invalid_steps_config = [
            {
                "sequence": [
                    {
                        "name": "compose_step",
                        "service": "muto_compose",
                        "plugin": "ComposePlugin",
                    },
                    {
                        "name": "provision_step",
                        "service": "muto_provision",
                        "plugin": "ProvisionPlugin",
                        "condition": "invalid expression!",
                    },
                ]
            }
        ]

        compensation_config = [
            {
                "name": "kill_step",
                "service": "muto_kill_stack",
                "plugin": "LaunchPlugin",
            }
        ]

        pipeline = Pipeline(
            name="test_pipeline",
            steps=invalid_steps_config,
            compensation=compensation_config,
        )

        def create_client_side_effect(plugin, service_name):
            client_mock = MagicMock()
            client_mock.wait_for_service.return_value = True
            future_mock = MagicMock()
            future_mock.result.return_value = MagicMock(success=True, err_msg="")
            client_mock.call_async.return_value = future_mock
            return client_mock

        mock_node = MagicMock(spec=Node)
        mock_create_node.return_value = mock_node
        mock_node.create_client.side_effect = create_client_side_effect

        pipeline.execute_pipeline()

        self.assertIn("compose_step", pipeline.context)
        self.assertTrue(pipeline.context["compose_step"].success)

        self.assertNotIn("provision_step", pipeline.context)

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
