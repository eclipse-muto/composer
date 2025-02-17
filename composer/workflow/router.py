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
