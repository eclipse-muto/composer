import os
import subprocess
import rclpy
from rclpy.node import Node
from muto_msgs.msg import StackManifest
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
            NativePlugin, "muto_native", self.handle_native
        )
        self.create_subscription(
            StackManifest, "composed_stack", self.handle_composed_stack, 10
        )
        self.current_stack: StackManifest | None = None
        self.archive = None
        self.deployment_path = "/var/tmp/muto_workspaces"

    def from_git(self, repo_url, branch="main"):
        if os.path.exists(self.deployment_path):
            os.chdir(self.deployment_path)
            cmd = f"cd {self.deployment_path} && git fetch origin && git checkout branch && git pull"
        else:
            os.mkdir(self.deployment_path)
            cmd = f"cd {self.deployment_path} && git clone -b {branch} {repo_url}"

        subprocess.run(cmd, shell=True, check=True)

    def from_tar(self, tar_file_path):
        if os.path.exists(self.deployment_path):
            subprocess.run(f"rm -rf {self.deployment_path}", shell=True, check=True)

        os.makedirs(self.deployment_path, exist_ok=True)
        cmd = f"tar -xzf {tar_file_path} -C {self.deployment_path}"
        subprocess.run(cmd, shell=True, check=True)

    def handle_native(self, request, response):
        try:
            if request.start:
                self.from_git(repo_url=self.current_stack.url, branch=self.current_stack.branch)
                self.find_launcher(ws_path=self.deployment_path, launcher_name=self.current_stack)
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

    def handle_git(self):
        try:
            # TODO:
            pass
        except Exception as e:
            raise Exception(f"Error while handling repo native: {e}")

    def find_launcher(self, ws_path, launcher_name):
        self.get_logger().info(f"walking in: {ws_path}")
        print("aaa")
        for root, dirs, files in os.walk(ws_path):
            if launcher_name in files:
                return os.path.join(root, launcher_name)
        return None


def main():
    rclpy.init()
    n = MutoDefaultNativePlugin()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
