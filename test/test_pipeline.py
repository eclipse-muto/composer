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
                        "name": "native_step",
                        "service": "muto_native",
                        "plugin": "NativePlugin",
                        "condition": "should_run_native == True",
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
        """
        Test that plugins are loaded successfully if they exist in the imported module.
        """
        # Mock the module returned by import_module
        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            NativePlugin=mock_plugin_class,
            LaunchPlugin=mock_plugin_class,
        )

        # Create the pipeline, which calls load_plugins in its constructor
        pipeline = Pipeline(
            name="test_pipeline",
            steps=self.steps_config,
            compensation=self.compensation_config,
        )

        # The pipeline’s plugins dictionary should contain all three plugins
        self.assertIn("ComposePlugin", pipeline.plugins)
        self.assertIn("NativePlugin", pipeline.plugins)
        self.assertIn("LaunchPlugin", pipeline.plugins)

    def test_load_plugins_failure(self):

        steps_config = [
            {
                "sequence": [
                    {
                        "name": "random_step",
                        "plugin": "RandomPlugin",
                        "service": "random_native",
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
        """
        Steps:
          - 'compose_step' always runs
          - 'native_step' and 'launch_step' are skipped if conditions are false
        """
        mock_spin.return_value = None  # no-op

        # Mock plugin classes
        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            NativePlugin=mock_plugin_class,
            LaunchPlugin=mock_plugin_class,
        )

        # We will create a new client mock for each service name
        def create_client_side_effect(plugin, service_name):
            client_mock = MagicMock()
            client_mock.wait_for_service.return_value = True

            # Future with a default success=True so steps pass if they run
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

        # Conditions that skip native & launch
        context = {"should_run_native": False, "should_run_launch": False}
        pipeline.execute_pipeline(additional_context=context)

        self.assertIn(
            "compose_step", pipeline.context, 
            "compose_step should be recorded in pipeline.context"
        )
        self.assertNotIn(
            "native_step", pipeline.context, 
            "native_step should not run, so not in pipeline.context"
        )
        self.assertNotIn(
            "launch_step", pipeline.context, 
            "launch_step should not run, so not in pipeline.context"
        )

    @patch("importlib.import_module")
    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_execute_pipeline_failure_triggers_compensation(
        self, mock_create_node, mock_spin, mock_import_module
    ):
        """
        Compose step succeeds, native step fails, triggers kill_step compensation.
        """
        mock_spin.return_value = None
        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            NativePlugin=mock_plugin_class,
            LaunchPlugin=mock_plugin_class,
        )

        # Distinct behavior for each service
        def create_client_side_effect(plugin, service_name):
            client_mock = MagicMock()
            client_mock.wait_for_service.return_value = True

            future_mock = MagicMock()

            if service_name == "muto_compose":
                # compose_step => success
                future_mock.result.return_value = MagicMock(success=True, err_msg="")
            elif service_name == "muto_native":
                # native_step => fails
                future_mock.result.return_value = MagicMock(success=False, err_msg="Native error")
            elif service_name == "muto_kill_stack":
                # kill_step => success
                future_mock.result.return_value = MagicMock(success=True, err_msg="")
            else:
                future_mock.result.return_value = MagicMock(success=True, err_msg="Unreached")

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

        # Trigger native step to run => it fails => triggers compensation
        context = {"should_run_native": True, "should_run_launch": True}
        pipeline.execute_pipeline(additional_context=context)

        # compose_step => success
        self.assertIn("compose_step", pipeline.context)
        self.assertTrue(pipeline.context["compose_step"].success)

        # native_step => fail
        self.assertIn("native_step", pipeline.context)
        self.assertFalse(pipeline.context["native_step"].success)

        # launch_step => never runs after native fails
        self.assertNotIn("launch_step", pipeline.context)

    @patch("importlib.import_module")
    @patch("rclpy.spin_until_future_complete")
    @patch("rclpy.create_node")
    def test_execute_pipeline_condition_evaluation_error(
        self, mock_create_node, mock_spin, mock_import_module
    ):
        """
        Invalid condition => triggers compensation => pipeline aborts.
        """
        mock_spin.return_value = None
        mock_plugin_class = MagicMock()
        mock_import_module.return_value = MagicMock(
            ComposePlugin=mock_plugin_class,
            NativePlugin=mock_plugin_class,
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
                        "name": "native_step",
                        "service": "muto_native",
                        "plugin": "NativePlugin",
                        # malformed condition
                        "condition": "invalid expression!",
                    },
                ]
            }
        ]

        compensation_config = [
            {"name": "kill_step", "service": "muto_kill_stack", "plugin": "LaunchPlugin"}
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

        # compose_step should succeed before the invalid condition is hit
        self.assertIn("compose_step", pipeline.context)
        self.assertTrue(pipeline.context["compose_step"].success)

        # Because the second step's condition is invalid, pipeline aborts
        # and triggers compensation step kill_step
        # You can check that kill_step was indeed attempted by mocking
        # calls or verifying logs; but the main assertion is that
        # 'native_step' does not appear because the condition parsing fails.
        self.assertNotIn("native_step", pipeline.context)

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
