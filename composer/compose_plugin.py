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
from rclpy.node import Node
from composer.model.stack import Stack
from composer.twin import Twin
from composer.model.edge_device import EdgeDevice
from muto_msgs.srv import ComposePlugin
from muto_msgs.msg import PluginResponse, StackManifest, PlanManifest

class MutoDefaultComposePlugin(Node):
    """
    Default composition plugin node for handling stack compositions
    """
    def __init__(self):
        super().__init__("compose_plugin")
        self._init_parameters()
        self._init_services()

    def _init_parameters(self):
        params = {
            "name": "example-01",
            "namespace": "org.eclipse.muto.sandbox",
            "stack_topic": "stack",
            "twin_topic": "twin",
            "anonymous": False,
            "twin_url": "http://ditto:ditto@sandbox.composiv.ai"
        }
        for param, value in params.items():
            self.declare_parameter(param, value)
        self.muto = {param: self.get_parameter(param).value for param in params}

        self.twin = Twin(node='muto_compose_plugin', config=self.muto, publisher=None)
        self.edge_device = EdgeDevice(twin=self.twin)

    def _init_services(self):
        self.create_service(ComposePlugin, "muto_compose", self.handle_compose)

    def handle_compose(self, req, res):
        """
        Handles composition requests, merging current and next stack definitions.

        :param req: The service request containing the composition plan.
        :param res: The service response to be filled with the result of the composition.
        :return: The service response with the composition result.
        """
        plan = req.input
        current_stack = Stack(self.edge_device, json.loads(plan.current.stack), None)
        next_stack_manifest = json.loads(plan.next.stack)

        next_stack = self._get_next_stack(next_stack_manifest)
        merged = current_stack.merge(next_stack)

        res.output = self._create_plan_manifest(plan, merged.manifest)
        return res

    def _get_next_stack(self, manifest):
        """
        Retrieves the next stack based on its manifest, handling direct definitions or references by ID.

        :param manifest: The manifest dictionary of the next stack.
        :return: An instance of the Stack class representing the next stack.
        """
        if 'stackId' in manifest:
            manifest = self.twin.stack(manifest['stackId'])
        return Stack(self.edge_device, manifest, None)

    def _create_plan_manifest(self, plan, merged_manifest):
        """
        Creates a PlanManifest message from the current and next plans, and the merged manifest.

        :param plan: The current composition plan.
        :param merged_manifest: The merged stack manifest.
        :return: A PlanManifest instance ready to be sent as a service response.
        """
        return PlanManifest(
            current=plan.current,
            next=plan.next,
            pipeline=plan.pipeline,
            planned=StackManifest(type="json", stack=json.dumps(merged_manifest)),
            result=PluginResponse(result_code=0, error_message="", error_description="")
        )

def main():
    rclpy.init()
    node = MutoDefaultComposePlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()