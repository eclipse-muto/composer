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

import os
import json
import yaml
import rclpy
from std_msgs.msg import String
from muto_msgs.msg import MutoAction
from composer.twin import Twin
from composer.router import Router
from composer.pipeline import Pipeline
from composer.model.edge_device import EdgeDevice
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class MutoComposer(Node):
    def __init__(self):
        super().__init__("muto_composer")
        self._init_parameters()
        self._init_resources()
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

    def _init_resources(self):
        """Initialize the resources required for the composer."""
        pipeline_file_path = os.path.join(
            get_package_share_directory("composer"), "config", "pipeline.yaml"
        )

        with open(pipeline_file_path, "r") as f:
            self.pipelines = yaml.safe_load(f)["pipelines"]

        self.twin_publisher = self.create_publisher(String, self.twin_topic, 10)
        self.create_subscription(MutoAction, self.stack_topic, self.on_stack_callback, 10)

    def _bootstrap(self):
        """Bootstrap the edge device, twin and pipelines."""
        try:
            self.twin = Twin(node='muto_composer', config=self.muto, publisher=self.twin_publisher)
            self.edge_device = EdgeDevice(twin=self.twin)
            self._init_pipelines()  
            self.edge_device.bootstrap()
            self.router = Router(self.edge_device, self.pipelines)
        except Exception as e:
            self.get_logger().error(f'An exception occurred in bootstrap: {e}')

    def _init_pipelines(self):
        """Initialize pipelines from configuration loaded from YAML."""
        loaded_pipelines = {} 

        for pipeline_item in self.pipelines:
            name = pipeline_item["name"]
            pipeline_spec = pipeline_item["pipeline"]
            compensation_spec = pipeline_item.get("compensation", None)
            
            # Create a Pipeline object for each pipeline item
            pipeline = Pipeline(self.edge_device, name, pipeline_spec, compensation_spec)
            loaded_pipelines[name] = pipeline

        self.pipelines = loaded_pipelines

    def on_stack_callback(self, msg):
        """
        Handle incoming MutoAction messages to route stack actions.

        :param msg: The MutoAction message containing the stack action and payload.
        """
        if msg:
            try:
                stack = json.loads(msg.payload)["value"]
                self.router.route(msg.method, stack)
            except KeyError as k:
                self.get_logger().error(f"Payload is not in the expected format: {k}")
            except Exception as e:
                self.get_logger().error(f"Invalid payload coming to muto composer: {e}")

def main(args=None):
    rclpy.init(args=args)
    muto_composer_node = MutoComposer()
    rclpy.spin(muto_composer_node)
    muto_composer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
