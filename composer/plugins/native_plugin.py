import os
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from muto_msgs.msg import StackManifest
from muto_msgs.srv import NativePlugin
from core.model.muto_archive import MutoArchive

WORKSPACES_PATH = os.path.join("/var", "tmp", "muto_workspaces")
GIT_UP_TO_DATE_MSG = "Your branch is up to date"

# TODO: When the version is already up to date, don't install rosdeps and don't build, else do


class MutoDefaultNativePlugin(Node):
    """The plugin for setting up the workspace (pull, clone, build, install dependendices, etc.)"""

    def __init__(self):
        super().__init__("native_plugin")
        self.native_srv = self.create_service(
            NativePlugin, "muto_native", self.handle_native
        )
        self.create_subscription(StackManifest, "composed_stack", self.get_stack, 10)

        self.current_stack: StackManifest | None = None
        self.is_up_to_date = False

    def get_stack(self, stack_msg: StackManifest):
        """Subscribe to the composed stack"""
        self.current_stack = stack_msg

    def from_git(self, repo_url, branch="main") -> None:
        """
        Clones a git repository if that repo does not exist within deployment path.
        If the repo exists, simply updates it.
        """
        target_dir = os.path.join(
            WORKSPACES_PATH, self.current_stack.name.replace(" ", "_")
        )
        if os.path.exists(os.path.join(target_dir, ".git")):
            os.chdir(target_dir)
            cmd = (
                f"git fetch --recurse-submodules origin && "
                f"git checkout {branch} && "
                f"git pull --recurse-submodules origin {branch} && "
                f"git submodule update --recursive --remote"
            )
        else:
            if os.path.exists(target_dir):
                shutil.rmtree(target_dir)
            os.makedirs(target_dir, exist_ok=True)
            cmd = (
                f"git clone --recurse-submodules -b {branch} {repo_url} {target_dir} && "
                f"cd {target_dir} && "
                f"git submodule foreach 'git checkout main || git checkout -b main origin/main' && "
                f"git submodule update --recursive --remote"
            )
        command = subprocess.run(cmd, shell=True, check=True, capture_output=True)
        self.is_up_to_date = GIT_UP_TO_DATE_MSG in command.stdout.decode()
        self.get_logger().info(f"command stdout: {command.stdout.decode()} Repo UPTODATE status: {self.is_up_to_date}")

    def from_tar(self, tar_file_path):
        """If the repo is a compressed file that needs to be unextracted"""
        archive = MutoArchive()

    def build_workspace(self):
        subprocess.run(
            [
                "colcon",
                "build",
                "--symlink-install",
                "--packages-ignore",
                "rototui_css",
                "--cmake-args",
                "-DCMAKE_BUILD_TYPE=Release",
            ],
            check=True,
        )

    def install_dependencies(self):
        subprocess.run(
            ["rosdep", "update"],
            check=True,
        )
        subprocess.run(
            ["rosdep", "install", "--from-path", ".", "--ignore-src", "-r", "-y"],
            check=True,
        )

    def handle_native(self, request, response):
        """Prepares the workspace"""
        try:
            if request.start:
                self.from_git(
                    repo_url=self.current_stack.url, branch=self.current_stack.branch
                )
                # if not self.is_up_to_date:  # TODO:
                self.install_dependencies()
                self.build_workspace()
            response.err_msg = str("Successful")
            response.success = True
        except Exception as e:
            self.get_logger().warn(f"Exception: {e}")
            response.err_msg = str(f"Error: {e}")
            response.success = False
        return response


def main():
    rclpy.init()
    n = MutoDefaultNativePlugin()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
