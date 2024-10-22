from composer.workflow.pipeline import Pipeline
import rclpy.logging

class Router():
    def __init__(self, pipelines):
        self.pipelines = pipelines

    def route(self, action):
        """Routes the action that is coming from agent"""
        rclpy.logging.get_logger("muto_router").info(f"Routing action: {action}")

        pipeline: Pipeline = self.pipelines.get(action)
        if pipeline:
            pipeline.execute_pipeline()
        else:
            rclpy.logging.get_logger("muto_router").warn(f"No pipeline found for action: {action}")
        