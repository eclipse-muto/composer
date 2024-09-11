import os
import rclpy
from rclpy.node import Node
from muto_msgs.msg import StackManifest, LocalMode, RepoMode
from muto_msgs.srv import NativePlugin
from core.model.muto_archive import MutoArchive

LOCAL_MODE = "local"
NATIVE_MODE = "native"
REPO_MODE = "repo"
CONTAINER_MODE = "container"


class MutoDefaultNativePlugin(Node):
    def __init__(self):
        super().__init__("native_plugin")
        self.native_srv = self.create_service(
            NativePlugin, 'muto_native', self.handle_native)
        self.create_subscription(
            StackManifest, 'composed_stack', self.handle_composed_stack, 10)
        self.local_pub = self.create_publisher(
            LocalMode, 'local_launch', 10)
        
        self.repo_pub = self.create_publisher(
            RepoMode, 'repo_launch', 10)
        self.current_stack: StackManifest | None = None
        self.archive = None

    def handle_native(self, request, response):
        try:
            if request.start:
                self.get_logger().info(f"Launch Mode: {self.current_stack.mode}")
                if self.current_stack.mode == NATIVE_MODE:
                    self.prep_native(self.current_stack.native.native_mode)
                elif self.current_stack.mode == CONTAINER_MODE:
                    self.get_logger().warn("Skipping NativePlugin as the stack is in container mode")
                else:
                    self.get_logger().warn("No mode provided. Skipping")
            response.err_msg = str()
            response.success = True
        except Exception as e:
            self.get_logger().warn(f"Exception: {e}")
            response.err_msg = str(e)
            response.success = False
        return response

    def handle_composed_stack(self, stack_msg: StackManifest):
        self.current_stack = stack_msg

    def prep_native(self, native_mode: str):
        if native_mode == REPO_MODE:
            self.handle_repo_native()
        elif native_mode == LOCAL_MODE:
            self.handle_local_native()
        else:
            self.get_logger().warn("Unknown native mode is provided")

    def handle_repo_native(self):
        try:
            self.archive = MutoArchive()
            ws_path = self.archive.download_workspace(
                url=self.current_stack.workspace_url)
            self.archive.decompress_into_local(ws_path)
            launcher_name = self.current_stack.native.repo.launch_file_name
            launcher_path = self.find_launcher(f"{os.path.join(os.path.expanduser('~'), 'muto_workspaces')}", launcher_name)
            if launcher_path:
                print(f"Launcher found: {launcher_path}")
            else:
                self.get_logger().warn(f"Launcher '{launcher_name}' not found in the workspace.")
            repo_msg = RepoMode()
            repo_msg.launch_file_name = launcher_path
            self.repo_pub.publish(repo_msg)

        except Exception as e:
            raise Exception(f"Error while handling repo native: {e}")

    def find_launcher(self, ws_path, launcher_name):
        print("walking in: ", ws_path)
        for root, dirs, files in os.walk(ws_path):
            if launcher_name in files:
                return os.path.join(root, launcher_name)
        return None


    def handle_local_native(self):
        try:
            ws_path = self.current_stack.native.local.ws_full_path
            os.chdir(ws_path)
            local_msg = LocalMode()
            local_msg.ws_full_path = ws_path
            local_msg.launcher_path_relative_to_ws = self.current_stack.native.local.launcher_path_relative_to_ws
            self.local_pub.publish(local_msg)
        except Exception as e:
            self.get_logger().info(
                f"Error while handling local native launch: {e}")


def main():
    rclpy.init()
    n = MutoDefaultNativePlugin()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
