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

import importlib
import json
import uuid
from muto_msgs.msg._stack_manifest import StackManifest
import rclpy
from rclpy.node import Node
import rclpy.logging
from composer.workflow.safe_evaluator import SafeEvaluator

class Pipeline:
    def __init__(self, name, steps, compensation):
        """
        Initializes the Pipeline with a name, steps, and compensation steps.

        Args:
            name (str): The name of the pipeline.
            steps (list): A list of steps to execute in the pipeline.
            compensation (list): A list of compensation steps to execute on failure.
        """
        self.name = name
        self.steps = steps
        self.compensation = compensation
        self.plugins = self.load_plugins()
        self.logger = rclpy.logging.get_logger(f"{self.name}_pipeline")
        self.context = {}  # To store step results

    def load_plugins(self) -> dict:
        """
        Load the plugins defined in the pipeline configuration.

        Returns:
            dict: A dictionary mapping plugin names to their corresponding classes.

        Raises:
            Exception: If a plugin class cannot be found in the module.
        """
        plugin_dict = {}
        try:
            module_name = "muto_msgs.srv"
            module = importlib.import_module(module_name)
        except ImportError as e:
            self.logger.error(f"Failed to import module '{module_name}': {e}")
            raise

        for item in self.steps:
            sequence = item.get("sequence", [])
            for step in sequence:
                plugin_name = step.get("plugin")
                if plugin_name and plugin_name not in plugin_dict:
                    try:
                        plugin_class = getattr(module, plugin_name)
                        plugin_dict[plugin_name] = plugin_class
                    except AttributeError:
                        self.logger.error(
                            f"Plugin '{plugin_name}' not found in '{module_name}'"
                        )
                        raise Exception(
                            f"Plugin '{plugin_name}' not found in module '{module_name}'. "
                            "Ensure the plugin has a corresponding service definition."
                        )
        for step in self.compensation:
            plugin_name = step.get("plugin")
            if plugin_name and plugin_name not in plugin_dict:
                try:
                    plugin_class = getattr(module, plugin_name)
                    plugin_dict[plugin_name] = plugin_class
                except AttributeError:
                    self.logger.error(
                        f"Compensation Plugin '{plugin_name}' not found in '{module_name}'"
                    )
                    raise Exception(
                        f"Compensation Plugin '{plugin_name}' not found in module '{module_name}'. "
                        "Ensure the plugin has a corresponding service definition."
                    )
        return plugin_dict

    def execute_pipeline(self, additional_context: dict = None, next_manifest=None):
        """
        Execute each pipeline step sequentially.
        If a step fails, execute compensation steps and abort the pipeline.
        Supports conditional execution based on step outcomes.

        Args:
            additional_context (dict): Additional context variables to include.
        """
        if additional_context:
            self.context.update(additional_context)

        executor = rclpy.create_node(f"{self.name}_pipeline_executor", enable_rosout=False)
        failed = False

        ## Request and responses are chained to pass stack manifests between steps
        ##.  initial request = xxx
        ##.  repeat all steps (nextstep):
        #        request = nextstep(request)
        input_manifest = self.toStackManifest(next_manifest)
        for item in self.steps:
            if failed:
                break
            sequence = item.get("sequence", [])
            for step in sequence:
                step_name = step.get("name")
                condition = step.get("condition")

                # Evaluate condition if present
                if condition:
                    evaluator = SafeEvaluator(self.context)
                    try:
                        should_execute = evaluator.eval_expr(condition)
                        self.logger.debug(f"Evaluating condition for step '{step_name}': {condition} => {should_execute}")
                        if not should_execute:
                            self.logger.info(f"Skipping step '{step_name}' due to condition: {condition}")
                            continue
                    except ValueError as e:
                        self.logger.error(f"Condition evaluation failed for step '{step_name}': {e}")
                        self.execute_compensation(executor)
                        failed = True
                        self.logger.error("Aborting the rest of the pipeline due to condition evaluation failure.")
                        break

                try:

                    response = self.execute_step(step, executor, inputManifest=input_manifest)
                    if not response:
                        response = type("Response", (), {"success": False, "err_msg": "No response from service."})()
                        self.logger.error(f"Step {step_name} failed due to no response.")
                    # Store the response in context regardless of success or failure
                    self.context[step_name] = type("Response", (), { "success": response.success, "err_msg": response.err_msg })()

                    if not response.success:
                        raise Exception(f"Step execution error: {response.err_msg}")
                    input_manifest = response.output.current
                    self.logger.info(f"Step passed: {step_name}")

                except Exception as e:
                    self.logger.warn(f"Step failed: {step_name}, Exception: {e}")
                    self.execute_compensation(executor)
                    failed = True
                    self.logger.error("Aborting the rest of the pipeline")
                    break


        executor.destroy_node()

    def execute_step(self, step, executor: Node, inputManifest=None):
        """
        Executes a single step using the appropriate ROS 2 service.

        Args:
            step (dict): The step configuration containing 'plugin' and 'service'.
            executor (Node): The ROS node used for service communication.

        Returns:
            The response from the service call.

        Raises:
            Exception: If the service call fails or required fields are missing.
        """
        plugin_name = step.get("plugin")
        service_name = step.get("service")

        if not plugin_name or not service_name:
            raise ValueError("Step must contain 'plugin' and 'service' fields.")

        plugin = self.plugins.get(plugin_name)
        if not plugin:
            raise Exception(f"Plugin '{plugin_name}' not loaded.")

        cli = executor.create_client(plugin, service_name)
        self.logger.info(f"Executing step: {plugin_name}")

        if not cli.wait_for_service(timeout_sec=5.0):
            self.logger.error(
                f"Service '{cli.srv_name}' is not available. Cannot execute step."
            )
            return None

        ## Request and responses are chained to pass stack manifests between steps
        ##.  initial request = xxx
        ##.  repeat all steps (nextstep):
        #        request = nextstep(request)
        req = plugin.Request()
        if inputManifest:
            req.input.current = inputManifest
        req.start = True
            
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(executor, future)

        if future.result():
            return future.result()
        else:
            raise Exception(f"Service call failed: {future.exception()}")

    def execute_compensation(self, executor: Node):
        """
        Executes compensation steps if the primary step execution fails.

        Args:
            executor (Node): The ROS node used for service communication.
        """
        self.logger.info("Executing compensation steps.")
        if self.compensation:
            for step in self.compensation:
                try:
                    self.execute_step(step, executor)
                except Exception as e:
                    self.logger.warn(
                        f"Compensation step failed: {step.get('plugin', '')}, Exception: {e}"
                    )
        else:
            self.logger.warn("No compensation steps to execute.")
    
    def toStackManifest(self, manifest):
        if manifest is None:
            return None
        stack_msg = StackManifest()
        # Handle both old format (name at root) and new format (metadata.name)
        if isinstance(manifest, dict):
            if 'metadata' in manifest and 'name' in manifest['metadata']:
                stack_msg.name = manifest['metadata']['name']
            else:
                stack_msg.name = manifest.get("name", "")
        stack_msg.stack = json.dumps(manifest)
        return stack_msg       
