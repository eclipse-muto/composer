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

import rclpy
from muto_msgs.srv import ProvisionPlugin
from .base_plugin import BasePlugin, StackOperation



WORKSPACES_PATH = os.path.join("/tmp", "muto", "muto_workspaces")
ARTIFACT_STATE_FILE = ".muto_artifact.json"


class MutoProvisionPlugin(BasePlugin):
    """Plugin for setting up the workspace (decode, extract, build, install dependencies, etc.)"""

    def __init__(self):
        super().__init__("provision_plugin")
        self.provision_srv = self.create_service(
            ProvisionPlugin, "muto_provision", self.handle_provision
        )

    def handle_provision(
        self, request: ProvisionPlugin.Request, response: ProvisionPlugin.Response
    ):
        """Service handler to prepare the workspace using double dispatch pattern."""
        handler, context = self.find_stack_handler(request)
        try:
            if handler and context:
                context.operation = StackOperation.PROVISION
                handler.apply_to_plugin(self, context, request, response)
                response.success = True
            else:
                response.err_msg = "No current stack received or start flag not set."
                response.success = False
                self.get_logger().warning("No current stack received or start flag not set.")
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            response.err_msg = f"Error: {e}"
            response.success = False
            
        response.output.current = request.input.current
        return response



def main():
    rclpy.init()
    provision_plugin = MutoProvisionPlugin()
    rclpy.spin(provision_plugin)
    provision_plugin.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
