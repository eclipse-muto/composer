#
#  Copyright (c) 2025 Composiv.ai, Eteration A.S. and others
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

import rclpy.logging
from composer.workflow.pipeline import Pipeline

class Router:
    def __init__(self, pipelines):
        """
        Initializes the Router with a dictionary of pipelines.

        Args:
            pipelines (dict): A dictionary mapping action names to Pipeline instances.
        """
        self.pipelines = pipelines
        self.logger = rclpy.logging.get_logger("muto_router")

    def route(self, action):
        """
        Routes the incoming action from the agent to the appropriate pipeline.

        Args:
            action (str): The action name to route.
        """
        self.logger.info(f"Routing action: {action}")

        pipeline: Pipeline = self.pipelines.get(action)
        if pipeline:
            pipeline.execute_pipeline()
        else:
            self.logger.warn(f"No pipeline found for action: {action}")
