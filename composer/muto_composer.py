import os
import re
import json
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from muto_msgs.msg import MutoAction
from muto_msgs.srv import CoreTwin
from ament_index_python.packages import get_package_share_directory
from composer.workflow.router import Router
from composer.workflow.pipeline import Pipeline

CORE_TWIN_NODE_NAME = "core_twin"


class MutoComposer(Node):
    def __init__(self):
        super().__init__("muto_composer")

        self.declare_parameter("stack_topic", "stack")
        self.incoming_stack_topic = (
            self.get_parameter("stack_topic").get_parameter_value().string_value
        )

        self.create_subscription(
            MutoAction, self.incoming_stack_topic, self.on_stack_callback, 10
        )
        self.get_stack_cli = self.create_client(
            CoreTwin, f"{CORE_TWIN_NODE_NAME}/get_stack_definition"
        )
        self.raw_stack_publisher = self.create_publisher(String, "raw_stack", 10)

        self.incoming_stack = None  # Incoming stack from agent
        self.method = None  # Action data coming from agent

        pipeline_file_path = os.path.join(
            get_package_share_directory("composer"), "config", "pipeline.yaml"
        )
        with open(pipeline_file_path, "r", encoding="utf-8") as f:
            pipeline_configs = yaml.safe_load(f)["pipelines"]

        self.init_pipelines(pipeline_configs)
        self.router = Router(self.pipelines)

    def init_pipelines(self, pipeline_configs):
        """Initialize pipelines that are loaded from pipeline.yaml file."""
        loaded_pipelines = {}

        for pipeline_item in pipeline_configs:
            name = pipeline_item["name"]
            pipeline_spec = pipeline_item["pipeline"]
            compensation_spec = pipeline_item.get("compensation", None)

            # Create a Pipeline object for each pipeline
            pipeline = Pipeline(name, pipeline_spec, compensation_spec)
            loaded_pipelines[name] = pipeline

        self.pipelines = loaded_pipelines

    def on_stack_callback(self, stack_msg: MutoAction):
        """
        Callback method for when a MutoAction message from the agent arrives.
        Parses the stackId and gets the stack using the core_twin's service.
        """
        try:
            self.method = stack_msg.method  # start, kill, apply, etc.
            payload = json.loads(stack_msg.payload)
            payload_value = payload["value"]
            stack_id = payload_value.get("stackId", "")
            req = CoreTwin.Request()
            req.input = stack_id
            future = self.get_stack_cli.call_async(req)
            future.add_done_callback(self.get_stack_done_callback)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in payload: {e}")
        except KeyError as k:
            self.get_logger().error(f"Payload is missing key: {k}")
        except Exception as e:
            self.get_logger().error(f"Error parsing stack from agent: {e}")

    def get_stack_done_callback(self, future):
        """
        Callback function executed when the service call is completed.
        Retrieves the result and routes the action from the agent.
        """
        try:
            result = future.result()
            if result:
                self.incoming_stack = result.output
                resolved_stack = self.resolve_expression(self.incoming_stack)
                self.publish_raw_stack(resolved_stack)
                if self.method:
                    self.router.route(action=self.method)
            else:
                self.get_logger().warn("Received empty result from service call.")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")

    def resolve_expression(self, value: str = "") -> str:
        """
        Resolve Muto expressions like $(find package_name) or $(env VAR_NAME).
        """
        expressions = re.findall(r'\$\(([\s0-9a-zA-Z_-]+)\)', value)
        result = value

        for expression in expressions:
            expr, var = expression.split()
            resolved_value = ""

            try:
                if expr == 'find':
                    resolved_value = get_package_share_directory(var)
                elif expr == 'env':
                    resolved_value = os.getenv(f'{var}', '')
                else:
                    self.get_logger().info("No muto expression found in the given string")
                result = re.sub(
                    r'\$\(' + re.escape(expression) + r'\)', resolved_value, result, count=1)
            except KeyError:
                self.get_logger().warn(f"{var} does not exist.")
                return result
            except Exception as e:
                self.get_logger().info(f'Exception occurred: {e}')
        return result

    def publish_raw_stack(self, stack: str):
        """Publish the received stack to the ROS environment."""
        stack_msg = String(data=stack)
        self.raw_stack_publisher.publish(stack_msg)


def main():
    rclpy.init()
    muto_composer = MutoComposer()
    rclpy.spin(muto_composer)
    muto_composer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
