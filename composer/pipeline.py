import importlib
import json
import rclpy
from rclpy.node import Node


class Pipeline():
    def __init__(self, name, steps, compensation) -> None:
        self.name = name
        self.steps = steps
        self.compensation = compensation
        self.plugins: dict = self.load_plugins()

    def load_plugins(self) -> dict:
        """Load the plugins that are defined in the pipeline.yaml file"""
        plugin_dict = {}
        for items in self.steps:
            sequence = items.get("sequence", [])
            for step in sequence:
                try:
                    plugin_name = step["plugin"]
                    module_name = "muto_msgs.srv"
                    module = importlib.import_module(module_name)
                    plugin_class = getattr(module, plugin_name)
                    plugin_dict[plugin_name] = plugin_class
                except Exception as e:
                    logger_node = rclpy.create_node("pipeline_logger")
                    logger_node.get_logger().error(
                        f"Exception while loading plugins: {e}")
                    logger_node.destroy_node()
                    raise Exception(
                        "Pipeline configuration and service definitions are not matching. A plugin should have the corrseponding srv definition")
        return plugin_dict

    def execute_pipeline(self):
        """
        Start executing each pipeline step seperately.
        Call compensation steps if an error occurs
        """
        executor = rclpy.create_node(f"{self.name}_pipeline_executor")
        failed = False
        for items in self.steps:
            if not failed:
                sequence = items.get("sequence", [])
                for step in sequence:
                    try:
                        response = self.execute_step(step, executor)
                        if not response:
                            raise Exception(
                                "No response from the service call. The service might not be up yet")

                        if not response.success:
                            raise Exception(f"Step execution error: {response.err_msg}")
                        executor.get_logger().info(f"Step passed: {step.get('plugin', '')}")

                    except Exception as e:
                        executor.get_logger().warn(
                            f'Step failed: {json.dumps(step["plugin"])}, Exception: {e}')
                        self.execute_compensation(executor)
                        failed = True
                        executor.get_logger().error("Aborting the rest of the pipeline")
                        break

        executor.destroy_node()

    def execute_step(self, step, executor: Node):
        """
        Executes a single step using the appropriate ROS 2 service.

        :return: The response from the service call.
        """
        plugin = self.plugins[step["plugin"]]
        service = step["service"]
        cli = executor.create_client(plugin, service)
        executor.get_logger().info(f"Executing: {step['plugin']}")
        if not cli.wait_for_service(timeout_sec=5.0):
            executor.get_logger().error(
                f'{cli.srv_name} service is not available. Can\'t execute step')
            return

        req = self.plugins[step["plugin"]].Request()
        req.start = True
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(executor, future)

        if future.result():
            return future.result()
        raise Exception(f"Service call failed: {future.exception()}")

    def execute_compensation(self, executor: Node):
        """
        Executes compensation steps if the primary step execution fails.
        """
        executor.get_logger().info('Executing compensation steps')
        if self.compensation:
            for step in self.compensation:
                try:
                    self.execute_step(step, executor)
                except Exception as e:
                    executor.get_logger().warn(
                        f'Compensation step failed: {json.dumps(step)}, Exception: {e}')
        else:
            executor.get_logger().warn("No compensation steps to execute.")
