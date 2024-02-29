#
#  Copyright (c) 2024 Composiv.ai, Eteration A.S. and others
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
#

import rclpy
from rclpy.node import Node
import json
from muto_msgs.msg import StackManifest, PlanManifest
from muto_msgs.srv import ComposePlugin

PLUGINS = {
    "ComposePlugin": ComposePlugin,
}

class Pipeline(Node):
    """
    Represents a pipeline for executing a series of actions or steps based on ROS 2 services.
    """
    def __init__(self, device, name, steps, compensation=None, node=None):
        """
        Initializes the pipeline node.

        :param device: The edge device associated with the pipeline.
        :param name: The name of the pipeline.
        :param steps: A list of steps (actions) to be executed in the pipeline.
        :param compensation: A list of compensation steps to be executed if a step fails.
        """
        super().__init__(f"{name}_pipeline_node")
        self.device = device
        self.name = name
        self.steps = steps
        self.compensation = compensation or []
        self.node = node

    def execute(self, command, current, next):
        """
        Executes the pipeline for the given command with the current and next stack configurations.

        :param command: The pipeline command to execute.
        :param current: The current stack configuration.
        :param next: The next stack configuration to apply.
        """
        self.get_logger().info(f'Executing pipeline for command: {command}')
        cstack = StackManifest(type="json", stack=json.dumps(current))
        pstack = StackManifest(type="json", stack=json.dumps(next))
        plan = PlanManifest(current=cstack, next=pstack, pipeline=command)

        pipeline = self.steps
        for items in pipeline:
            sequence = items.get("sequence", [])
            for step in sequence:
                try:
                    response = self.execute_step(plan, step)
                    if response.output.result.result_code != 0:
                        raise Exception("Step execution error")
                    plan = response.output
                    self.get_logger().info(f'Step passed: {json.dumps(step)}')
                except Exception as e:
                    self.get_logger().warn(f'Step failed: {json.dumps(step)}, Exception: {e}')
                    self.execute_compensation(plan)
                    return

    def execute_step(self, plan, step):
        """
        Executes a single step using the appropriate ROS 2 service.

        :param plan: The plan manifest for the step execution.
        :param step: The step details including the service and plugin to use.
        :return: The response from the service call.
        """
        cli = self.create_client(PLUGINS[step["plugin"]], step["service"])
        self.wait_for_service(cli)
        req = PLUGINS[step["plugin"]].Request()
        req.input = plan
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Service call failed: {future.exception()}")

    def execute_compensation(self, plan):
        """
        Executes compensation steps if the primary step execution fails.

        :param plan: The plan manifest for the compensation execution.
        """
        self.get_logger().info('Executing compensation steps')
        for step in self.compensation:
            try:
                self.execute_step(plan, step)
            except Exception as e:
                self.get_logger().warn(f'Compensation step failed: {json.dumps(step)}, Exception: {e}')

    def wait_for_service(self, client, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise Exception(f"Service {client.srv_name} not available")
