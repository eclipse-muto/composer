import os
import json
import subprocess
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import rclpy
from muto_msgs.msg import StackManifest
from muto_msgs.srv import LaunchPlugin, CoreTwin
from composer.workflow.launcher import Ros2LaunchParent
from launch import LaunchService



class MutoDefaultLaunchPlugin(Node):
    def __init__(self):
        super().__init__("launch_plugin")
        self.start_srv = self.create_service(
            LaunchPlugin, "muto_start_stack", self.handle_start
        )

        self.stop_srv = self.create_service(
            LaunchPlugin, "muto_kill_stack", self.handle_kill
        )

        self.apply_srv = self.create_service(
            LaunchPlugin, "muto_apply_stack", self.handle_apply
        )

        self.create_subscription(
            StackManifest, "composed_stack", self.handle_composed_stack, 10
        )

        self.set_stack_cli = self.create_client(CoreTwin, "core_twin/set_current_stack")

        self.launcher_name = None

        self.current_stack = None
        self.launcher_path = None
        self.ws_full_path = "/var/tmp/muto_workspaces"
        self.launcher_full_path = None
        self.launch_arguments = None
        self.launch_description = None
        self.launch_service: LaunchService | None = None
        self.launcher = Ros2LaunchParent(self.launch_arguments)

        self.async_loop = asyncio.get_event_loop()
        self.timer = self.create_timer(
            0.1, self.run_async_loop, callback_group=ReentrantCallbackGroup()
        )

    def run_async_loop(self):
        """Periodically step through the asyncio event loop."""
        self.async_loop.stop()
        self.async_loop.run_forever()

    def handle_composed_stack(self, stack_msg: StackManifest):
        try:
            self.current_stack = stack_msg
            args = json.loads(self.current_stack.args)
            self.launch_arguments = [
                f"{str(key)}:={str(value)}" for key, value in args.items()
            ]
        except Exception as e:
            self.get_logger().info(f"Exception while parsing the arguments: {e}")

    def source_workspaces(self):
        """Source the given workspaces within muto session and update the environment."""
        sources = json.loads(self.current_stack.source)

        for key, val in sources.items():
            self.get_logger().info(f"Sourcing: {key} | {val}")
            command = f'bash -c "source {val} && env"'
            proc = subprocess.Popen(
                command, stdout=subprocess.PIPE, shell=True, executable="/bin/bash"
            )
            for line in proc.stdout:
                line = line.decode("utf-8").strip()
                key, _, value = line.partition("=")
                if key and value:
                    os.environ[key] = value
            proc.communicate()

    def handle_start(self, request, response):
        try:
            if request.start:
                os.chdir(self.ws_full_path)
                for i in self.launch_arguments:
                    self.get_logger().info(f"Argument: {i}")
                self.source_workspaces()
                if self.launcher_full_path:
                    task = self.async_loop.create_task(
                        self.launcher.launch_a_launch_file(
                            launch_file_path=self.launcher_full_path,
                            launch_file_arguments=self.launch_arguments,
                        )
                    )

                    task.add_done_callback(self.on_launch_done)
                elif True:
                    os.chdir(
                        os.path.join("/var/tmp", "muto_workspaces")
                    )  # Go to ws path
                    self.get_logger().info(f"Current working directory: {os.getcwd()}")
                    self.get_logger().info(f"Launcher path: {self.launcher_path}")
                    self.build_workspace()
                    self.source_workspaces()
                    task = self.async_loop.create_task(
                        self.launcher.launch_a_launch_file(
                            launch_file_path=self.launcher_path,
                            launch_file_arguments=self.launch_arguments,
                        )
                    )
                    response.success = True
                    response.err_msg = ""

        except Exception as e:
            self.get_logger().warn(f"Exception occurred: {e}")
            response.err_msg = str(e)
            response.success = False
        return response

    def on_launch_done(self, future):
        try:
            self.launch_description, self.launch_service = future.result()
            self.get_logger().info("Launch completed successfully!")
        except Exception as e:
            self.get_logger().warn(f"Launch failed: {e}")

    def build_workspace(self):
        subprocess.run(
            [
                "colcon",
                "build",
                "--symlink-install",
                "--cmake-args",
                "-DCMAKE_BUILD_TYPE=Release",
            ],
            check=True,
        )

    def handle_kill(self, request, response):
        try:
            if request.start:
                if self.current_stack:
                    req = CoreTwin.Request()
                    stack_id = self.current_stack.stack_id
                    req.input = stack_id
                    future = self.set_stack_cli.call_async(req)
                    future.add_done_callback(self.set_stack_done_callback)
                    self.launcher.kill()
                    response.success = True
                    response.err_msg = ""
                else:
                    self.get_logger().error("No composed stack. Aborting")
            return response
        except Exception as e:
            self.get_logger().info(f"Exception occurred: {e}")
            response.success = False
            response.err_msg = f"{e}"

    def handle_apply(self, request, response):
        if request.start:
            # TODO:
            self.get_logger().info("Handling apply")
        response.err_msg = ""
        response.success = True
        return response

    def set_stack_done_callback(self, future):
        """
        Callback function for the future object.

        This callback function is executed when the service call is completed.
        It retrieves the result from the Future object and routes the action that comes from agent

        Args:
            future: The Future object representing the service call.
        """
        result = future.result()
        if result:
            self.get_logger().info("Edge device stack setting is done successfully")
        else:
            self.get_logger().warn(
                "Edge Device stack setting failed. Try your request again."
            )


def main():
    rclpy.init()
    l = MutoDefaultLaunchPlugin()
    rclpy.spin(l)
    l.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
