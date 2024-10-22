import json
import rclpy
from muto_msgs.msg import StackManifest, NativeMode, RepoMode, LocalMode
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
                stack_msg = StackManifest()
                stack_msg.name = self.incoming_stack.get("name", "")
                stack_msg.stack_id = self.incoming_stack.get("stackId", "")
                stack_msg.context = self.incoming_stack.get("context", "")
                stack_msg.mode = self.incoming_stack.get("mode", "")
                stack_msg.args = json.dumps(self.incoming_stack.get("args", {}))
                stack_msg.workspace_url = self.incoming_stack.get("url", "https://")
                native = self.incoming_stack.get("native", {})
                stack_msg.native = NativeMode()
                stack_msg.native.native_mode = native.get("native_mode", "")
                stack_msg.native.repo = RepoMode()
                stack_msg.native.repo.path_to_download_relative_to_home_dir = (
                    native.get("repo", {}).get(
                        "path_to_download_relative_to_home_dir", ""
                    )
                )
                stack_msg.native.repo.launch_file_name = native.get("repo", {}).get(
                    "launch_file_name", ""
                )
                stack_msg.native.local = LocalMode()
                stack_msg.native.local.ws_full_path = native.get("local", {}).get(
                    "ws_full_path", ""
                )
                stack_msg.native.local.launcher_path_relative_to_ws = native.get(
                    "local", {}
                ).get("launcher_path_relative_to_ws", "")
                stack_msg.source = json.dumps(self.incoming_stack.get("source", {}))
                self.composed_stack_publisher.publish(stack_msg)
        except Exception as e:
            raise Exception(f"Incoming stack format is not valid: {e}")


def main():
    rclpy.init()
    c = MutoDefaultComposePlugin()
    rclpy.spin(c)
    c.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
