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

import json
import rclpy
from std_msgs.msg import String
from muto_msgs.srv import ComposePlugin
from muto_msgs.msg import PluginResponse, PlanManifest
from rclpy.node import Node
import composer.twin as twin
import composer.model.edge_device as edge

class MutoDefaultLaunchPlugin(Node):
    """
    A default launch plugin for handling stack operations like apply, kill, and start within the Muto system.
    """
    
    def __init__(self):
        super().__init__("launch_plugin")
        self._init_parameters()
        self._init_services()
        self._bootstrap()

    def _init_parameters(self):
        """Declare and retrieve node parameters."""
        params = {
            "name": "example-01",
            "namespace": "org.eclipse.muto.sandbox",
            "stack_topic": "stack",
            "twin_topic": "twin",
            "anonymous": False,
            "twin_url": "http://ditto:ditto@sandbox.composiv.ai"
        }
        for param, default in params.items():
            self.declare_parameter(param, default)
        self.muto = {param: self.get_parameter(param).value for param in params}
        self.twin_topic = self.muto['twin_topic']
        self.stack_topic = self.muto['stack_topic']

    def _init_services(self):
        """Initializes services for apply, kill, and start stack operations."""
        self.create_service(ComposePlugin, "muto_apply_stack", self.handle_apply)
        self.create_service(ComposePlugin, "muto_kill_stack", self.handle_kill)
        self.create_service(ComposePlugin, "muto_start_stack", self.handle_start)

    def _bootstrap(self):
        """Bootstraps the twin and edge device components."""
        self.twin = twin.Twin(node='MutoLaunchPlugin', config=self.muto, publisher=None)
        self.device = edge.EdgeDevice(twin=self.twin)

    def handle_apply(self, req, res):
        """Handles requests to apply changes to a stack."""
        return self._handle_stack_operation(req, res, self.device.apply)

    def handle_kill(self, req, res):
        """Handles requests to kill a stack."""
        return self._handle_stack_operation(req, res, self.device.kill)

    def handle_start(self, req, res):
        """Handles requests to start a stack."""
        return self._handle_stack_operation(req, res, self.device.activate)

    def _handle_stack_operation(self, req, res, operation_method):
        """
        Handles stack operations by performing the given operation method.

        Args:
            req: The request object containing the stack information.
            res: The response object to be populated based on the operation outcome.
            operation_method: The method to be called for the stack operation.

        Returns:
            The response object with the operation result.
        """
        try:
            if req.input.planned:
                stack = json.loads(req.input.planned.stack)
                operation_method(stack)
                res.output = self._success_response(req.input.planned)
            else:
                res.output = self._error_response("Could not handle launch")
        except Exception as e:
            self.get_logger().error(f'Error handling stack operation: {e}')
            res.output = self._error_response("Exception during operation")
        return res

    def _success_response(self, planned):
        """Creates a success response for a stack operation."""
        return PlanManifest(
            result=PluginResponse(result_code=0, error_message="", error_description=""),
            planned=planned
        )

    def _error_response(self, error_message):
        """Creates an error response for a stack operation."""
        return PlanManifest(
            result=PluginResponse(result_code=1000, error_message=error_message, error_description=error_message)
        )

def main(args=None):
    rclpy.init(args=args)
    node = MutoDefaultLaunchPlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
