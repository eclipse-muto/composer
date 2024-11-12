import os
import json
import subprocess
import asyncio
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from muto_msgs.msg import StackManifest
from muto_msgs.srv import LaunchPlugin, CoreTwin
from composer.workflow.launcher import Ros2LaunchParent
from composer.plugins.native_plugin import WORKSPACES_PATH


class MutoDefaultLaunchPlugin(Node):
    def __init__(self):
        super().__init__("launch_plugin")

        # Initialize ROS 2 services
        self.start_srv = self.create_service(
            LaunchPlugin, "muto_start_stack", self.handle_start
        )
        self.stop_srv = self.create_service(
            LaunchPlugin, "muto_kill_stack", self.handle_kill
        )
        self.apply_srv = self.create_service(
            LaunchPlugin, "muto_apply_stack", self.handle_apply
        )

        # Subscription to receive the composed stack
        self.create_subscription(StackManifest, "composed_stack", self.get_stack, 10)

        # Client to set the current stack
        self.set_stack_cli = self.create_client(CoreTwin, "core_twin/set_current_stack")

        # Initialize attributes
        self.current_stack: Optional[StackManifest] = None
        self.launch_arguments = []
        self.launcher = Ros2LaunchParent(self.launch_arguments)

        # Set up asyncio event loop
        self.async_loop = asyncio.get_event_loop()
        self.timer = self.create_timer(
            0.1, self.run_async_loop, callback_group=ReentrantCallbackGroup()
        )

    def run_async_loop(self):
        """Periodically step through the asyncio event loop."""
        self.async_loop.stop()
        self.async_loop.run_forever()

    def get_stack(self, stack_msg: StackManifest):
        """Callback to receive the composed stack and parse launch arguments."""
        try:
            self.current_stack = stack_msg
            args = json.loads(self.current_stack.args)
            self.launch_arguments = [f"{key}:={value}" for key, value in args.items()]
            self.get_logger().info("Parsed launch arguments from stack.")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing launch arguments: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def find_file(self, ws_path: str, file_name: str) -> Optional[str]:
        """
        Helper method to find a file in the workspace path.

        Args:
            ws_path (str): The workspace path to search in.
            file_name (str): The name of the file to find.

        Returns:
            Optional[str]: The full path to the file if found, otherwise None.
        """
        self.get_logger().info(f"Searching for {file_name} under {ws_path}")
        for root, _, files in os.walk(ws_path):
            if file_name in files:
                found_path = os.path.join(root, file_name)
                self.get_logger().info(f"Found file: {found_path}")
                return found_path
        self.get_logger().warning(f"File '{file_name}' not found under '{ws_path}'.")
        return None

    def source_workspaces(self):
        """
        Source the specified workspaces and update the environment variables.
        """
        if not self.current_stack:
            self.get_logger().error("No current stack available to source workspaces.")
            return

        sources = json.loads(self.current_stack.source)
        workspace_dir = os.path.join(
            WORKSPACES_PATH, self.current_stack.name.replace(" ", "_")
        )

        for name, source_script in sources.items():
            self.get_logger().info(f"Sourcing: {name} | {source_script}")
            command = f"bash -c 'source {source_script} && env'"
            try:
                result = subprocess.run(
                    command,
                    stdout=subprocess.PIPE,
                    shell=True,
                    executable="/bin/bash",
                    cwd=workspace_dir,
                    check=True,
                    text=True,
                )
                env_output = result.stdout
                env_vars = dict(
                    line.split("=", 1)
                    for line in env_output.splitlines()
                    if "=" in line
                )
                os.environ.update(env_vars)
                self.get_logger().info(f"Sourced workspace: {name}")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(
                    f"Failed to source workspace '{name}': {e.stderr}"
                )
            except Exception as e:
                self.get_logger().error(f"Error sourcing workspace '{name}': {e}")

    def handle_start(
        self, request: LaunchPlugin.Request, response: LaunchPlugin.Response
    ):
        """
        Service handler for starting the stack.

        Args:
            request (LaunchPlugin.Request): The service request.
            response (LaunchPlugin.Response): The service response.

        Returns:
            LaunchPlugin.Response: The response indicating success or failure.
        """
        try:
            if request.start:
                for arg in self.launch_arguments:
                    self.get_logger().info(f"Launch Argument: {arg}")

                self.source_workspaces()

                if self.current_stack and self.current_stack.launch_description_source:
                    launch_file = self.find_file(
                        os.path.join(
                            WORKSPACES_PATH, self.current_stack.name.replace(" ", "_")
                        ),
                        self.current_stack.launch_description_source,
                    )
                    if not launch_file:
                        raise FileNotFoundError(
                            f"Launch file not found: {self.current_stack.launch_description_source}"
                        )

                    # Schedule the coroutine in the event loop
                    asyncio.run_coroutine_threadsafe(
                        self.launcher.launch_a_launch_file(
                            launch_file_path=launch_file,
                            launch_file_arguments=self.launch_arguments,
                        ),
                        self.async_loop,
                    )
                    self.get_logger().info("Launch file execution initiated.")
                elif self.current_stack and self.current_stack.on_start:
                    script = self.find_file(
                        os.path.join(
                            WORKSPACES_PATH, self.current_stack.name.replace(" ", "_")
                        ),
                        self.current_stack.on_start,
                    )
                    if not script:
                        raise FileNotFoundError(
                            f"Script not found: {self.current_stack.on_start}"
                        )

                    self.run_script(script)
                    self.get_logger().info("Start script executed successfully.")
                else:
                    self.get_logger().warning(
                        "No launch description or start script provided."
                    )
                    response.success = False
                    response.err_msg = (
                        "No launch description or start script provided."
                    )
                    return response

                response.success = True
                response.err_msg = ""
            else:
                response.success = False
                response.err_msg = "Start flag not set in request."
                self.get_logger().warning("Start flag not set in start request.")
        except Exception as e:
            self.get_logger().error(f"Exception occurred during start: {e}")
            response.err_msg = str(e)
            response.success = False
        return response

    def run_script(self, script_path: str):
        """Run a script with proper error handling."""
        if not os.path.isfile(script_path):
            raise FileNotFoundError(f"Script not found: {script_path}")

        if not os.access(script_path, os.X_OK):
            os.chmod(script_path, 0o755)

        try:
            result = subprocess.run(
                [script_path], check=True, capture_output=True, text=True
            )
            self.get_logger().info(f"Script output: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Script failed with error: {e.stderr}")
            raise

    def handle_kill(
        self, request: LaunchPlugin.Request, response: LaunchPlugin.Response
    ):
        """
        Service handler for killing the stack.

        Args:
            request (LaunchPlugin.Request): The service request.
            response (LaunchPlugin.Response): The service response.

        Returns:
            LaunchPlugin.Response: The response indicating success or failure.
        """
        try:
            if request.start:
                if self.current_stack:
                    req = CoreTwin.Request()
                    req.input = self.current_stack.stack_id
                    future = self.set_stack_cli.call_async(req)
                    future.add_done_callback(self.set_stack_done_callback)

                    if self.current_stack.launch_description_source:
                        self.launcher.kill()
                        self.get_logger().info("Launch process killed successfully.")
                    elif self.current_stack.on_kill:
                        script = self.find_file(
                            os.path.join(
                                WORKSPACES_PATH,
                                self.current_stack.name.replace(" ", "_"),
                            ),
                            self.current_stack.on_kill,
                        )
                        if not script:
                            raise FileNotFoundError(
                                f"Script not found: {self.current_stack.on_kill}"
                            )

                        self.run_script(script)
                        self.get_logger().info("Kill script executed successfully.")
                    else:
                        self.get_logger().warning(
                            "No launch description or kill script provided."
                        )
                        response.success = False
                        response.err_msg = (
                            "No launch description or kill script provided."
                        )
                        return response

                    response.success = True
                    response.err_msg = "Handle kill success"
                else:
                    self.get_logger().error(
                        "No composed stack available. Aborting kill operation."
                    )
                    response.success = False
                    response.err_msg = "No composed stack available."
            else:
                response.success = False
                response.err_msg = "Start flag not set in request."
                self.get_logger().warning("Start flag not set in kill request.")
        except Exception as e:
            self.get_logger().error(f"Exception occurred during kill: {e}")
            response.err_msg = str(e)
            response.success = False
        return response

    def handle_apply(
        self, request: LaunchPlugin.Request, response: LaunchPlugin.Response
    ):
        """
        Service handler for applying the stack configuration.

        Args:
            request (LaunchPlugin.Request): The service request.
            response (LaunchPlugin.Response): The service response.

        Returns:
            LaunchPlugin.Response: The response indicating success or failure.
        """
        try:
            if request.start:
                # TODO: Implement apply logic
                self.get_logger().info("Handling apply operation.")
                response.success = True
                response.err_msg = ""
            else:
                response.success = False
                response.err_msg = "Start flag not set in request."
                self.get_logger().warning("Start flag not set in apply request.")
        except Exception as e:
            self.get_logger().error(f"Exception occurred during apply: {e}")
            response.err_msg = str(e)
            response.success = False
        return response

    def set_stack_done_callback(self, future):
        """Callback function executed when the set_current_stack service call is completed."""
        try:
            result = future.result()
            if result:
                self.get_logger().info(
                    "Edge device stack setting completed successfully."
                )
            else:
                self.get_logger().warning(
                    "Edge device stack setting failed. Please try your request again."
                )
        except Exception as e:
            self.get_logger().error(f"Exception in set_stack_done_callback: {e}")


def main():
    rclpy.init()
    launch_plugin = MutoDefaultLaunchPlugin()
    rclpy.spin(launch_plugin)
    launch_plugin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
