import os
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from muto_msgs.msg import StackManifest
from muto_msgs.srv import NativePlugin

WORKSPACES_PATH = os.path.join("/var", "tmp", "muto_workspaces")


class MutoDefaultNativePlugin(Node):
    """Plugin for setting up the workspace (clone, update, build, install dependencies, etc.)"""

    def __init__(self):
        super().__init__("native_plugin")

        self.current_stack: StackManifest | None = None
        self.is_up_to_date = False

        self.declare_parameter("ignored_packages", [""])
        self.ignored_packages = [
            pkg
            for pkg in self.get_parameter("ignored_packages")
            .get_parameter_value()
            .string_array_value
            if pkg
        ]

        self.create_subscription(StackManifest, "composed_stack", self.get_stack, 10)
        self.native_srv = self.create_service(
            NativePlugin, "muto_native", self.handle_native
        )

    def get_stack(self, stack_msg: StackManifest):
        """Callback to receive the composed stack."""
        self.current_stack = stack_msg

    def from_git(self, repo_url: str, branch: str = "main") -> None:
        """
        Clone or update a git repository, including its submodules.

        Args:
            repo_url (str): The URL of the git repository.
            branch (str): The branch to check out.
        """
        if not self.current_stack:
            self.get_logger().error("No current stack available.")
            return

        target_dir = os.path.join(
            WORKSPACES_PATH, self.current_stack.name.replace(" ", "_")
        )

        if os.path.exists(os.path.join(target_dir, ".git")):
            self.update_repository(target_dir, branch)
        else:
            self.clone_repository(repo_url, target_dir, branch)

        submodules_up_to_date = self.checkout_and_check_submodules(target_dir, branch)
        self.is_up_to_date = self.is_up_to_date and submodules_up_to_date

    def clone_repository(self, repo_url: str, target_dir: str, branch: str):
        """Clone the repository and check out the specified branch."""
        if os.path.exists(target_dir):
            shutil.rmtree(target_dir)
        os.makedirs(target_dir, exist_ok=True)

        subprocess.run(
            ["git", "clone", "--recurse-submodules", repo_url, target_dir],
            check=True,
        )

        self.checkout_branch(target_dir, branch)
        self.is_up_to_date = False

    def update_repository(self, target_dir: str, branch: str):
        """Update the existing repository."""
        # Fetch updates from remote
        subprocess.run(
            ["git", "fetch", "--recurse-submodules", "origin"],
            check=True,
            cwd=target_dir,
        )

        # Check if local and remote commits differ
        try:
            local_commit = subprocess.check_output(
                ["git", "rev-parse", "@"],
                text=True,
                cwd=target_dir,
            ).strip()
            remote_commit = subprocess.check_output(
                ["git", "rev-parse", "@{u}"],
                text=True,
                cwd=target_dir,
            ).strip()

            if local_commit != remote_commit:
                self.is_up_to_date = False
            else:
                self.is_up_to_date = True
        except subprocess.CalledProcessError:
            self.get_logger().warning("HEAD not set. Checking out branch.")
            self.checkout_branch(target_dir, branch)
            self.is_up_to_date = False

        if not self.is_up_to_date:
            self.get_logger().info("Main repo is not up-to-date. Pulling changes.")
            subprocess.run(
                ["git", "checkout", branch],
                check=True,
                cwd=target_dir,
            )
            subprocess.run(
                ["git", "pull"],
                check=True,
                cwd=target_dir,
            )

    def checkout_branch(self, repo_dir: str, branch: str):
        """Check out the specified branch, falling back to 'main' if necessary."""
        try:
            subprocess.run(
                ["git", "checkout", branch],
                check=True,
                cwd=repo_dir,
            )
        except subprocess.CalledProcessError:
            self.get_logger().warning(
                f"Branch '{branch}' not found. Falling back to 'main'."
            )
            subprocess.run(
                ["git", "checkout", "main"],
                check=True,
                cwd=repo_dir,
            )

    def checkout_and_check_submodules(
        self, target_dir: str, branch: str = "main"
    ) -> bool:
        """
        Ensure all submodules are checked out to the specified branch and are up-to-date.

        Args:
            target_dir (str): The path to the main repository.
            branch (str): The branch to check out in submodules.

        Returns:
            bool: True if all submodules are up-to-date, False otherwise.
        """
        all_submodules_up_to_date = True

        try:
            submodules = (
                subprocess.check_output(
                    ["git", "submodule", "--quiet", "foreach", "echo $sm_path"],
                    text=True,
                    cwd=target_dir,
                )
                .strip()
                .split("\n")
            )

            for submodule in submodules:
                submodule_path = os.path.join(target_dir, submodule)
                self.checkout_branch(submodule_path, branch)

                try:
                    local_commit = subprocess.check_output(
                        ["git", "rev-parse", "@"],
                        text=True,
                        cwd=submodule_path,
                    ).strip()
                    remote_commit = subprocess.check_output(
                        ["git", "rev-parse", "@{u}"],
                        text=True,
                        cwd=submodule_path,
                    ).strip()

                    if local_commit != remote_commit:
                        all_submodules_up_to_date = False
                        self.get_logger().info(
                            f"Submodule '{submodule}' is not up-to-date. Pulling changes."
                        )
                        subprocess.run(
                            ["git", "pull"],
                            check=True,
                            cwd=submodule_path,
                        )
                except subprocess.CalledProcessError:
                    self.get_logger().warning(
                        f"Submodule '{submodule}' is not properly set up."
                    )
                    all_submodules_up_to_date = False

        except subprocess.CalledProcessError as e:
            self.get_logger().warning(f"Failed to checkout or update submodules: {e}")
            all_submodules_up_to_date = False

        return all_submodules_up_to_date

    def from_tar(self, tar_file_path: str):
        """Extract a compressed tar file (Not implemented)."""
        # Placeholder for future implementation

    def build_workspace(self):
        """Build the workspace using colcon."""
        colcon_command = [
            "colcon",
            "build",
            "--symlink-install",
        ]

        if self.ignored_packages:
            colcon_command += ["--packages-ignore"] + self.ignored_packages

        colcon_command += ["--cmake-args", "-DCMAKE_BUILD_TYPE=Release"]

        try:
            subprocess.run(
                colcon_command,
                check=True,
                cwd=self.get_workspace_dir(),
            )
        except subprocess.CalledProcessError:
            self.get_logger().warning("Build failed. Cleaning and retrying...")
            self.clean_build_workspace()
            subprocess.run(
                colcon_command,
                check=True,
                cwd=self.get_workspace_dir(),
            )

    def clean_build_workspace(self):
        """Remove build and install directories to clean the workspace."""
        workspace_dirs = ["build", "install"]
        for dir_name in workspace_dirs:
            dir_path = os.path.join(self.get_workspace_dir(), dir_name)
            if os.path.exists(dir_path):
                shutil.rmtree(dir_path)
                self.get_logger().info(
                    f"Removed {dir_name} directory to clean build workspace."
                )

    def install_dependencies(self):
        """Install dependencies using rosdep."""
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
                cwd=self.get_workspace_dir(),
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().warning(f"Failed to install dependencies: {e}")
            raise

    def handle_native(
        self, request: NativePlugin.Request, response: NativePlugin.Response
    ):
        """Service handler to prepare the workspace."""
        try:
            if request.start:
                if self.current_stack:
                    self.from_git(
                        repo_url=self.current_stack.url,
                        branch=self.current_stack.branch,
                    )
                    if not self.is_up_to_date:
                        self.get_logger().info(
                            "Workspace is NOT up-to-date. Updating..."
                        )
                        self.install_dependencies()
                        self.build_workspace()
                    else:
                        self.get_logger().info(
                            "Workspace is up-to-date. Skipping build and provisioning steps."
                        )
                    response.err_msg = "Successful"
                    response.success = True
                else:
                    response.err_msg = "No current stack received."
                    response.success = False
                    self.get_logger().error("No current stack received.")
            else:
                response.err_msg = "Start flag not set in request."
                response.success = False
                self.get_logger().warning(
                    "Start flag not set in native plugin request."
                )
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            response.err_msg = f"Error: {e}"
            response.success = False
        return response

    def get_workspace_dir(self) -> str:
        """Get the workspace directory for the current stack."""
        if not self.current_stack:
            self.get_logger().error("No current stack available.")
            return ""
        return os.path.join(WORKSPACES_PATH, self.current_stack.name.replace(" ", "_"))


def main():
    rclpy.init()
    native_plugin = MutoDefaultNativePlugin()
    rclpy.spin(native_plugin)
    native_plugin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
