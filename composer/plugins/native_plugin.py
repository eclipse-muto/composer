import os
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from muto_msgs.msg import StackManifest
from muto_msgs.srv import NativePlugin
from core.model.muto_archive import MutoArchive

WORKSPACES_PATH = os.path.join("/var", "tmp", "muto_workspaces")


class MutoDefaultNativePlugin(Node):
    """The plugin for setting up the workspace (pull, clone, build, install dependendices, etc.)"""

    def __init__(self):
        super().__init__("native_plugin")
        self.native_srv = self.create_service(
            NativePlugin, "muto_native", self.handle_native
        )
        self.create_subscription(StackManifest, "composed_stack", self.get_stack, 10)
        self.declare_parameter("ignored_packages", [""])
        self.ignored_packages = (
            self.get_parameter("ignored_packages")
            .get_parameter_value()
            .string_array_value
        )

        self.current_stack: StackManifest | None = None
        self.is_up_to_date = False

    def get_stack(self, stack_msg: StackManifest):
        """Subscribe to the composed stack"""
        self.current_stack = stack_msg

    def from_git(self, repo_url, branch="main") -> None:
        """
        Clones a git repository if that repo does not exist within deployment path.
        If the repo exists, simply updates it.
        Handles cases where the branch is not found or HEAD is not properly set.
        Ensures submodules are checked out and up-to-date with the specified branch.
        """
        target_dir = os.path.join(
            WORKSPACES_PATH, self.current_stack.name.replace(" ", "_")
        )

        if os.path.exists(os.path.join(target_dir, ".git")):
            os.chdir(target_dir)
            # Fetch updates from remote
            subprocess.run(
                "git fetch --recurse-submodules origin", shell=True, check=True
            )

            # Check if HEAD exists, otherwise set it
            try:
                local_commit = subprocess.check_output(
                    "git rev-parse @", shell=True, text=True
                ).strip()
                remote_commit = subprocess.check_output(
                    "git rev-parse @{u}", shell=True, text=True
                ).strip()

                if local_commit != remote_commit:
                    self.is_up_to_date = False
                else:
                    self.is_up_to_date = True
            except subprocess.CalledProcessError:
                self.get_logger().warn("HEAD not set. Checking out branch.")
                subprocess.run(f"git checkout {branch}", shell=True, check=True)
                self.is_up_to_date = False

            if not self.is_up_to_date:
                self.get_logger().info(f"Main repo is not up-to-date. Pulling changes.")
                subprocess.run(
                    f"git checkout {branch} && git pull", shell=True, check=True
                )

        else:
            if os.path.exists(target_dir):
                shutil.rmtree(target_dir)
            os.makedirs(target_dir, exist_ok=True)

            # Clone and ensure branch is checked out
            subprocess.run(
                f"git clone --recurse-submodules {repo_url} {target_dir}",
                shell=True,
                check=True,
            )
            os.chdir(target_dir)
            try:
                subprocess.run(f"git checkout {branch}", shell=True, check=True)
            except subprocess.CalledProcessError:
                self.get_logger().warn(
                    f"Branch '{branch}' not found. Falling back to 'main'."
                )
                subprocess.run("git checkout main", shell=True, check=True)
            self.is_up_to_date = False

        submodules_up_to_date = self.checkout_and_check_submodules(target_dir, branch)
        self.is_up_to_date = self.is_up_to_date and submodules_up_to_date

    def checkout_and_check_submodules(self, target_dir, branch="main"):
        """
        Ensures that all submodules are checked out to the specified branch and checks if they are up-to-date.
        """
        all_submodules_up_to_date = True

        try:
            submodules = (
                subprocess.check_output(
                    "git submodule --quiet foreach 'echo $sm_path'",
                    shell=True,
                    text=True,
                )
                .strip()
                .split("\n")
            )

            for submodule in submodules:
                submodule_path = os.path.join(target_dir, submodule)
                os.chdir(submodule_path)

                try:
                    subprocess.run(f"git checkout {branch}", shell=True, check=True)
                except subprocess.CalledProcessError:
                    self.get_logger().warn(
                        f"Submodule '{submodule}': Branch '{branch}' not found. Falling back to 'main'."
                    )
                    subprocess.run("git checkout main", shell=True, check=True)

                try:
                    submodule_local_commit = subprocess.check_output(
                        "git rev-parse @", shell=True, text=True
                    ).strip()
                    submodule_remote_commit = subprocess.check_output(
                        "git rev-parse @{u}", shell=True, text=True
                    ).strip()

                    if submodule_local_commit != submodule_remote_commit:
                        all_submodules_up_to_date = False
                        self.get_logger().info(
                            f"Submodule '{submodule}' is not up-to-date. Pulling changes."
                        )
                        subprocess.run("git pull", shell=True, check=True)
                except subprocess.CalledProcessError:
                    self.get_logger().warn(
                        f"Submodule '{submodule}' is not properly set up."
                    )
                    all_submodules_up_to_date = False

                os.chdir(
                    target_dir
                ) 

        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"Failed to checkout or update submodules: {e}")
            all_submodules_up_to_date = False

        return all_submodules_up_to_date

    def from_tar(self, tar_file_path):
        """If the repo is a compressed file that needs to be unextracted"""
        archive = MutoArchive()

    def build_workspace(self):
        try:
            ignored_packages_list = [pkg for pkg in self.ignored_packages if pkg]

            colcon_command = [
                "colcon",
                "build",
                "--symlink-install",
            ]

            if ignored_packages_list:
                colcon_command += ["--packages-ignore"] + ignored_packages_list

            colcon_command += ["--cmake-args", "-DCMAKE_BUILD_TYPE=Release"]

            subprocess.run(
                colcon_command,
                check=True,
            )
        except subprocess.CalledProcessError:
            self.get_logger().warn("Build failed. Cleaning and retrying...")
            self.clean_build_workspace()

            subprocess.run(
                colcon_command,
                check=True,
            )

    def clean_build_workspace(self):
        """Removes build, install, and log directories to clean workspace."""
        workspace_dirs = ["build", "install"]
        for dir_name in workspace_dirs:
            dir_path = os.path.join(
                WORKSPACES_PATH, self.current_stack.name.replace(" ", "_"), dir_name
            )
            if os.path.exists(dir_path):
                shutil.rmtree(dir_path)
                self.get_logger().info(
                    f"Removed {dir_name} directory to clean build workspace."
                )

    def install_dependencies(self):
        try:
            subprocess.run(
                ["rosdep", "update"],
                check=True,
            )
            subprocess.run(
                [
                    "rosdep",
                    "install",
                    "--from-path",
                    ".",
                    "--ignore-src",
                    "-r",
                    "-y",
                ],
                check=True,
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"Failed to install dependencies: {e}")
            raise

    def handle_native(self, request, response):
        """Prepares the workspace"""
        try:
            if request.start:
                self.from_git(
                    repo_url=self.current_stack.url, branch=self.current_stack.branch
                )
                if not self.is_up_to_date:
                    self.get_logger().info("Workspace is NOT up to date. Updating...")
                    self.install_dependencies()
                    self.build_workspace()
                else:
                    self.get_logger().info(
                        "Workspace is up to date. Skipping building and provising steps."
                    )
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
