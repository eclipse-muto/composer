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
from .base_plugin import BasePlugin, StackOperation

import rclpy
from std_msgs.msg import String
from muto_msgs.msg import StackManifest
from muto_msgs.srv import ComposePlugin

class MutoDefaultComposePlugin(BasePlugin):
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
            # Check if this is a kill action - kill actions pass through compose without validation
            is_kill_action = False
            if request.input.current.stack:
                try:
                    stack_data = json.loads(request.input.current.stack)
                    # Kill actions have stackId in value key or at top level, not a full manifest
                    if stack_data.get("value", {}).get("stackId") or (
                        stack_data.get("path", "").endswith("/kill") and not stack_data.get("launch")
                    ):
                        is_kill_action = True
                except json.JSONDecodeError:
                    pass

            if is_kill_action:
                # Kill actions don't need compose validation - just pass through
                self.get_logger().info("Kill action detected - skipping compose validation")
                response.success = True
                response.output.current = request.input.current
                return response

            handler, context = self.find_stack_handler(request)
            if not handler or not context:
                response.success = False
                response.err_msg = "No valid handler or context found."
                self.get_logger().warn("No valid handler or context found.")
            else:
                context.operation = StackOperation.COMPOSE
                handler.apply_to_plugin(self, context, request, response)
                response.success = True

        except Exception as e:
            response.success = False
            response.err_msg = str(e)
            self.get_logger().error(f"Exception during compose: {e}")

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