import json
import rclpy
from muto_msgs.msg import StackManifest
from muto_msgs.srv import ComposePlugin
from std_msgs.msg import String
from rclpy.node import Node

COMPOSE_PLUGIN_NODE_NAME = "compose_plugin"


class MutoDefaultComposePlugin(Node):
    def __init__(self):
        super().__init__(COMPOSE_PLUGIN_NODE_NAME)
        self.incoming_stack = None
        self.composed_stack_publisher = self.create_publisher(
            StackManifest, "composed_stack", 10
        )
        self.create_subscription(String, "raw_stack", self.handle_raw_stack, 10)
        self.compose_srv = self.create_service(
            ComposePlugin, "muto_compose", self.handle_compose
        )

    def handle_raw_stack(self, stack_msg: String):
        self.incoming_stack = json.loads(stack_msg.data)

    def handle_compose(self, request, response):
        try:
            if request.start:
                self.publish_composed_stack()
                response.success = True
                response.err_msg = str()
        except Exception as e:
            response.success = False
            response.err_msg = str(e)
        return response

    def publish_composed_stack(self):
        """Publish the end result of the composed stack to the ROS environment"""
        try:
            if self.incoming_stack:
                stack_msg = self.parse_stack(self.incoming_stack)
                self.composed_stack_publisher.publish(stack_msg)
        except Exception as e:
            raise Exception(f"Incoming stack format is not valid: {e}")
    
    def parse_stack(self, stack: dict) -> StackManifest:
        """Parses the incoming stack into StackManifest msg"""
        stack_msg = StackManifest()
        stack_msg.name = stack.get("name", "")
        stack_msg.context = stack.get("context", "")
        stack_msg.stack_id = stack.get("stackId", "")
        stack_msg.url = stack.get("url", "")
        stack_msg.branch = stack.get("branch", "")
        stack_msg.launch_description_source = stack.get("launch_description_source", "")
        stack_msg.on_start = stack.get("on_start", "")
        stack_msg.on_kill = stack.get("on_kill", "")
        stack_msg.args = json.dumps(stack.get("args", {}))
        stack_msg.source = json.dumps(stack.get("source", {}))
        return stack_msg


def main():
    rclpy.init()
    c = MutoDefaultComposePlugin()
    rclpy.spin(c)
    c.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
