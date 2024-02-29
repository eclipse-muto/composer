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

class Router:
    """
    Routes actions to the appropriate pipeline for execution based on the provided action and payload.
    """

    def __init__(self, device, pipelines):
        """
        Initializes the Router with a device and a set of pipelines.

        :param device: The device instance that the router will operate on.
        :param pipelines: A dictionary mapping action names to Pipeline objects.
        """
        self.device = device
        self.pipelines = pipelines

    def route(self, action, payload):
        """
        Routes the given action and payload to the appropriate pipeline for execution.

        :param action: The action to be executed, which corresponds to a key in the pipelines dictionary.
        :param payload: The payload associated with the action, typically representing the desired state or configuration.
        """
        print(f"Routing action: {action} with payload: {payload}")

        pipeline = self.pipelines.get(action)
        if pipeline is not None:
            self.device.bootstrap()
            current_stack = self.device.current_stack
            manifest = current_stack.manifest if current_stack else {}
            pipeline.execute(action, manifest, payload)
        else:
            print(f"No pipeline found for action: {action}")
