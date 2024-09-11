from composer.pipeline import Pipeline

class Router():
    def __init__(self, pipelines):
        self.pipelines = pipelines

    def route(self, action):
        """Routes the action that is coming from agent"""
        print(f"Routing action: {action}")

        pipeline: Pipeline = self.pipelines.get(action)
        if pipeline:
            pipeline.execute_pipeline()
        else:
            print(f"No pipeline found for action: {action}")
        