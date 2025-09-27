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

import os
import re
import json
import yaml
import base64
from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
from muto_msgs.msg import MutoAction
from muto_msgs.srv import CoreTwin
from ament_index_python.packages import get_package_share_directory
from composer.workflow.router import Router
from composer.workflow.pipeline import Pipeline
from rclpy.task import Future
from composer.workflow.schemas.pipeline_schema import PIPELINE_SCHEMA
from jsonschema import validate, ValidationError
from composer.model.stack import Stack
from composer.utils.stack_parser import create_stack_parser

CORE_TWIN_NODE_NAME = "core_twin"


class MutoComposer(Node):
    def __init__(self):
        super().__init__("muto_composer")

        self.declare_parameter("stack_topic", "stack")
        self.declare_parameter("twin_url", "sandbox.composiv.ai")
        self.declare_parameter("namespace", "org.eclipse.muto.sandbox")
        self.declare_parameter("name", "example-01")

        self.twin_url = (
            self.get_parameter("twin_url").get_parameter_value().string_value
        )
        self.twin_namespace = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )
        self.name = self.get_parameter("name").get_parameter_value().string_value
        self.next_stack_topic = (
            self.get_parameter("stack_topic").get_parameter_value().string_value
        )

        self.create_subscription(
            MutoAction, self.next_stack_topic, self.on_stack_callback, 10
        )

        self.bootstrap_pub = self.create_publisher(
            MutoAction, self.next_stack_topic, 10
        )
        self.raw_stack_publisher = self.create_publisher(String, "raw_stack", 10)
        self.current_stack_publisher = self.create_publisher(
            String, "current_stack", 10
        )
        self.next_stack_publisher = self.create_publisher(String, "next_stack", 10)

        self.get_stack_cli = self.create_client(
            CoreTwin, f"{CORE_TWIN_NODE_NAME}/get_stack_definition"
        )
        self.set_stack_cli = self.create_client(CoreTwin, f"{CORE_TWIN_NODE_NAME}/set_current_stack")

        self.current_stack = None
        self.next_stack = None  # Next stack to be processed
        self.method = None  # Action data coming from agent
        self.thing_id = f"{self.twin_namespace}:{self.name}"

        # Load pipeline configuration
        pipeline_file_path = os.path.join(
            get_package_share_directory("composer"), "config", "pipeline.yaml"
        )
        pipeline_config = self.load_pipeline_config(pipeline_file_path)
        self.init_pipelines(pipeline_config["pipelines"])
        # self.router = Router(self.pipelines)

        # Initialize stack parser utility
        self.stack_parser = create_stack_parser(self.get_logger())

        # Bootstrap
        # DO NOT GET THE CURRENT STACK FROM
        # TWIN, IT MAY NOT BE THE DEFAULT STACK
        # SYMPHONY STATES WILL HANDLE THAT
        #self.bootstrap()

    def bootstrap(self):
        """
        Bootstrap the device by activating the default stack.
        """
        try:
            req = CoreTwin.Request()
            res = requests.get(
                f"{self.twin_url}/api/2/things/{self.thing_id}/features/stack/properties/current",
                headers={"Content-type": "application/json"},
            )
            stack_id = res.json().get("stackId", "")
            req.input = stack_id
            future = self.get_stack_cli.call_async(req)
            future.add_done_callback(self.activate)
        except AttributeError:
            self.get_logger().error("No default stack. Aborting bootstrap")
        except Exception as e:
            self.get_logger().error(f"Error while bootstrapping: {e}")

    def activate(self, future: Future):
        """
        Callback to handle the response from the CoreTwin service during bootstrap.
        """
        try:
            result = future.result()
            if result:
                self.current_stack = json.loads(result.output)
                resolved_stack = self.resolve_expression(
                    json.dumps(self.current_stack)
                )
                self.publish_current_stack(resolved_stack)
                self.publish_raw_stack(resolved_stack)
                self.pipeline_execute("start", None, json.loads(resolved_stack))
            else:
                self.get_logger().error(
                    "No default stack received. Aborting bootstrap."
                )
        except AttributeError:
            self.get_logger().error("No default stack. Aborting bootstrap")
        except Exception as e:
            self.get_logger().error(f"Error while bootstrapping: {e}")

    def load_pipeline_config(self, file_path):
        """
        Load and validate the pipeline configuration from a YAML file.
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        try:
            validate(instance=config, schema=PIPELINE_SCHEMA)
        except ValidationError as e:
            raise ValueError(f"Invalid pipeline configuration: {e}")
        return config

    def init_pipelines(self, pipeline_config):
        """
        Initialize pipelines that are loaded from pipeline.yaml file.
        """
        loaded_pipelines = {}

        for pipeline_item in pipeline_config:
            name = pipeline_item["name"]
            pipeline_spec = pipeline_item["pipeline"]
            compensation_spec = pipeline_item.get("compensation", None)

            # Create a Pipeline object for each pipeline
            pipeline = Pipeline(name, pipeline_spec, compensation_spec)
            loaded_pipelines[name] = pipeline

        self.pipelines = loaded_pipelines

    def on_stack_callback(self, stack_msg: MutoAction):
        """
        Callback method for when a MutoAction message from the agent arrives.
        Parses the stackId and gets the stack using the core_twin's service.
        """
        try:
            self.method = stack_msg.method  # start, kill, apply
            payload = json.loads(stack_msg.payload)

            # Use the new stack parser utility to handle different payload formats
            parsed_stack = self.stack_parser.parse_payload(payload)
            if parsed_stack and parsed_stack != payload:
                payload = parsed_stack
                self.get_logger().info("Parsed stack from payload using stack parser utility.")
           
            # if the payload has a value key, extract stackId from it
            # otherwise, assume the payload is the stack itself do not get
            # stack from core_twin, use the payload directly

            if "value" in payload:
                payload_value = payload["value"]
                stack_id = payload_value.get("stackId", "")
                req = CoreTwin.Request()
                req.input = stack_id
                future = self.get_stack_cli.call_async(req)
                future2 = self.set_stack_cli.call_async(req)
                future2.add_done_callback(self.set_stack_done_callback)
                future.add_done_callback(self.get_stack_done_callback)
            else:
                # Use payload directly as the stack
                resolved_stack = self.resolve_expression(json.dumps(payload))
                self.next_stack = resolved_stack
                self.determine_execution_path()
                self.publish_next_stack(self.next_stack)
                self.publish_raw_stack(resolved_stack)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in payload: {e}")
        except KeyError as k:
            self.get_logger().error(f"Payload is missing key: {k}")
        except Exception as e:
            self.get_logger().error(f"Error parsing stack from agent: {e}")

    def set_stack_done_callback(self, future):
        """Callback function executed when the set_current_stack service call is completed."""
        try:
            result = future.result()
            if result:
                self.get_logger().info(
                    "Edge device stack setting completed successfully."
                )
            else:
                self.get_logger().warning(
                    "Edge device stack setting failed. Please try your request again."
                )
        except Exception as e:
            self.get_logger().error(f"Exception in set_stack_done_callback: {e}")

    def get_stack_done_callback(self, future):
        """
        Callback function executed when the service call is completed.
        Retrieves the result and routes the action from the agent.
        """
        try:
            result = future.result()
            if result:
                next_stack = json.loads(result.output)
                resolved_stack = self.resolve_expression(
                    json.dumps(next_stack)
                )

                self.next_stack = resolved_stack
                self.determine_execution_path()
                self.publish_next_stack(self.next_stack)
                self.publish_raw_stack(resolved_stack)
            else:
                self.get_logger().warn("Received empty result from service call.")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")

    def publish_current_stack(self, stack: str):
        """Publish the current stack to the ROS environment."""
        stack_msg = String(data=stack)
        self.current_stack_publisher.publish(stack_msg)

    def publish_next_stack(self, stack: str):
        """Publish the next stack to the ROS environment."""
        stack_msg = String(data=stack)
        self.next_stack_publisher.publish(stack_msg)

    def publish_raw_stack(self, stack: str):
        """Publish the received stack to the ROS environment."""
        stack_msg = String(data=stack)
        self.raw_stack_publisher.publish(stack_msg)

    def determine_execution_path(self):
        """
        Determines execution path and merge stacks based on stack attributes.
        """
        if not self.next_stack:
            self.get_logger().info("Waiting for the next stack.")
            return

        try:
            next_stack = json.loads(self.next_stack)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse next stack JSON: {e}")
            return

        self.get_logger().info(f"Next stack keys: {list(next_stack.keys())}")

        is_next_stack_empty = not next_stack.get("node", "") and not next_stack.get(
            "composable", ""
        )

        has_launch_description = bool(next_stack.get("launch_description_source"))
        has_on_start_and_on_kill = all(
            [next_stack.get("on_start"), next_stack.get("on_kill")]
        )
        
        has_archive_artifact = next_stack.get("metadata", {}).get("content_type", "") == "stack/archive"
        has_json_artifact = next_stack.get("metadata", {}).get("content_type", "") == "stack/json"
        artifact_present = next_stack
        if has_archive_artifact:
            self.get_logger().info(f"Artifact details detected: {artifact_present.keys() if isinstance(artifact_present, dict) else artifact_present}")

        if has_archive_artifact:
            should_run_provision = True
            should_run_launch = True
            self.get_logger().info(
                "Archive manifest detected; running ProvisionPlugin and LaunchPlugin."
            )
            self.current_stack = next_stack

        elif has_json_artifact:
            should_run_provision = False
            should_run_launch = True
            ns = next_stack.get("launch", {})
            csmeta = None 
            if self.current_stack is not None:
                csmeta = self.current_stack.get("metadata", None)
            if csmeta is not None and isinstance(csmeta, dict):
                merged_stack = self.merge(self.current_stack.get("launch", {}), ns)
                next_stack["launch"] = merged_stack
                self.current_stack = next_stack
            else:
                merged_stack = self.merge(self.current_stack, ns)
                next_stack["launch"] = merged_stack
                self.current_stack = next_stack
               
            self.publish_raw_stack(json.dumps(next_stack))  # Publish the merged stack

            self.get_logger().info(
                "JSON manifest detected; running LaunchPlugin."
            )
        elif is_next_stack_empty and (has_launch_description or has_on_start_and_on_kill):
            # Condition to run ProvisionPlugin
            should_run_provision = False
            should_run_launch = True
            self.get_logger().info(
                "Legacy stack conditions met to run LaunchPlugin."
            )
        elif not is_next_stack_empty:
            # Condition to merge stacks and bypass ProvisionPlugin
            should_run_provision = False
            should_run_launch = True
            self.get_logger().info(
                "Conditions met to merge stacks and bypass ProvisionPlugin."
            )
            # Merge current and next stacks
            merged_stack = self.merge(self.current_stack, next_stack)
            self.current_stack = merged_stack
            self.publish_raw_stack(json.dumps(merged_stack))  # Publish the merged stack
        else:
            # Conditions not met to run ProvisionPlugin
            should_run_provision = False
            should_run_launch = False
            self.get_logger().info(
                "Conditions not met to run ProvisionPlugin AND LaunchPlugin."
            )

        # Execute the appropriate pipeline with context variables
        execution_context = {
            "should_run_provision": should_run_provision,
            "should_run_launch": should_run_launch,
        }
        self.pipeline_execute(self.method, execution_context, self.current_stack)

    def merge(self, current_stack: dict, next_stack: dict) -> dict:
        """
        Merge current and next stack dictionaries.

        Args:
            current_stack (dict): The current stack data.
            next_stack (dict): The next stack data.

        Returns:
            dict: The merged stack data.
        """
        cs = current_stack
        if current_stack is None:
            cs = {}

        stack_1 = Stack(manifest=cs)
        stack_2 = Stack(manifest=next_stack)
        merged = stack_1.merge(stack_2)
        return merged.manifest

    def pipeline_execute(self, pipeline_name: str, additional_context: dict = None, stack_manifest=None):
        """
        Execute a specific pipeline by name with additional context.

        Args:
            pipeline_name (str): The name of the pipeline to execute.
            additional_context (dict): Additional context variables for conditions.
        """
        pipeline = self.pipelines.get(pipeline_name)
        if pipeline:
            self.get_logger().info(
                f"Executing pipeline: {pipeline_name} with context: {additional_context}"
            )
            pipeline.execute_pipeline(additional_context=additional_context, next_manifest=stack_manifest)
        else:
            self.get_logger().warn(f"No pipeline found with name: {pipeline_name}")

    def resolve_expression(self, value: str = "") -> str:
        """
        Resolve Muto expressions like $(find package_name) or $(env VAR_NAME).
        """
        expressions = re.findall(r"\$\(([\s0-9a-zA-Z_-]+)\)", value)
        result = value

        for expression in expressions:
            parts = expression.split()
            if len(parts) != 2:
                self.get_logger().warning(f"Invalid expression format: {expression}")
                continue
            expr, var = parts
            resolved_value = ""

            try:
                if expr == "find":
                    resolved_value = get_package_share_directory(var)
                elif expr == "env":
                    resolved_value = os.getenv(f"{var}", "")
                elif expr == "arg":
                    self.get_logger().info(f"Parsing {expr}: {var}")
                    resolved_value = self.current_stack.get("args", {}).get(var, "")
                    self.get_logger().info(f"Resolved arg: {resolved_value}")
                else:
                    self.get_logger().info(
                        "No muto expression found in the given string"
                    )
                result = re.sub(
                    r"\$\(" + re.escape(expression) + r"\)",
                    resolved_value,
                    result,
                    count=1,
                )
            except KeyError:
                self.get_logger().warn(f"{var} does not exist.")
                continue
            except Exception as e:
                self.get_logger().info(f"Exception occurred: {e}")
                continue
        return result


def main(args=None):
    rclpy.init(args=args)
    composer = MutoComposer()
    rclpy.spin(composer)
    composer.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
