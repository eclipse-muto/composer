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

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from muto_msgs.msg import StackManifest
from muto_msgs.srv import ComposePlugin
from composer.model.stack import Stack


class MutoDefaultComposePlugin(Node):
    def __init__(self):
        super().__init__("compose_plugin")

        self.incoming_stack = None
        self.next_stack = None  # To store the next stack if needed

        self.create_subscription(String, "raw_stack", self.handle_raw_stack, 10)
        self.composed_stack_publisher = self.create_publisher(
            StackManifest, "composed_stack", 10
        )
        self.compose_srv = self.create_service(
            ComposePlugin, "muto_compose", self.handle_compose
        )

    def handle_raw_stack(self, stack_msg: String):
        """
        Callback to handle incoming raw stack messages.
        Parses the JSON data and stores it.
        """
        try:
            stack_data = json.loads(stack_msg.data)
            self.incoming_stack = stack_data
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse raw stack JSON: {e}")

    def handle_compose(
        self, request: ComposePlugin.Request, response: ComposePlugin.Response
    ):
        """
        Service handler for composing the stack.
        Publishes the composed stack if 'start' is True.
        """
        try:
            if request.start:
                if self.incoming_stack:
                    self.publish_composed_stack()
                    response.success = True
                    response.err_msg = ""
                else:
                    response.success = True
                    response.err_msg = "No default stack."
                    self.get_logger().warn("No stack to compose.")
            else:
                response.success = False
                response.err_msg = "Start flag not set in request."
                self.get_logger().warn("Start flag not set in compose request.")
        except Exception as e:
            response.success = False
            response.err_msg = str(e)
            self.get_logger().error(f"Exception during compose: {e}")
        
        ## Simply chain the input to putput for now..
        ## This plugin should be able to determine how 
        ## the pipeline will continue to work i.e. apply policies
        ## and transformations to the stack.
        response.output.current = request.input.current
        return response

    def publish_composed_stack(self):
        """
        Publish the composed stack to the ROS environment.
        """
        stack_msg = self.parse_stack(self.incoming_stack)
        self.composed_stack_publisher.publish(stack_msg)

    def parse_stack(self, stack: dict) -> StackManifest:
        """
        Parses the incoming stack dictionary into a StackManifest message.

        Args:
            stack (dict): The stack data as a dictionary.

        Returns:
            StackManifest: The parsed stack as a ROS message.
        """
        stack_msg = StackManifest()
        stack_msg.name = stack.get("name", "")
        stack_msg.context = stack.get("context", "")
        stack_msg.stack_id = stack.get("stackId", "")
        stack_msg.url = stack.get("url", "")
        stack_msg.branch = stack.get("branch", "")
        stack_msg.launch_description_source = stack.get("launch_description_source", "")
        stack_msg.on_start = stack.get("on_start", "")
        stack_msg.on_kill = stack.get("on_kill", "")
        stack_msg.args = json.dumps(stack.get("args", {}))
        stack_msg.source = json.dumps(stack.get("source", {}))
        stack_msg.stack = json.dumps(stack)
        return stack_msg


def main():
    rclpy.init()
    compose_plugin = MutoDefaultComposePlugin()
    rclpy.spin(compose_plugin)
    compose_plugin.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()