#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

import os
import json
import subprocess
import asyncio
import atexit
from typing import Dict, Union, Optional
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from muto_msgs.msg import StackManifest
from muto_msgs.srv import LaunchPlugin, CoreTwin
from composer.workflow.launcher import Ros2LaunchParent
from composer.utils.paths import WORKSPACES_PATH
from composer.utils.stack_parser import StackParser
from .base_plugin import BasePlugin, StackContext, StackOperation


class ShellProcessWrapper:
    """
    Wrapper for subprocess.Popen to provide a compatible interface with Ros2LaunchParent
    for lifecycle management in _managed_launchers dictionary.
    """

    def __init__(self, process: subprocess.Popen, script_path: str):
        self.process = process
        self.script_path = script_path

    def kill(self) -> None:
        """Terminate the shell process and its process group."""
        if self.process.poll() is None:  # Process is still running
            try:
                # Kill the entire process group (handles child processes)
                os.killpg(os.getpgid(self.process.pid), 9)
            except (ProcessLookupError, OSError):
                # Process already terminated
                pass
            finally:
                self.process.wait()


class MutoDefaultLaunchPlugin(BasePlugin):
    # Process health monitoring interval in seconds
    PROCESS_MONITOR_INTERVAL = 1.0

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

        self.set_stack_cli = self.create_client(CoreTwin, "core_twin/set_current_stack")

        self.stack_parser = StackParser(self.get_logger())

        # Track managed launchers by launch file for consistent lifecycle management
        # Supports both Ros2LaunchParent (for .launch.py) and ShellProcessWrapper (for .sh)
        self._managed_launchers: Dict[str, Union[Ros2LaunchParent, ShellProcessWrapper]] = dict()

        # Track stack name associated with each launcher for crash reporting
        self._launcher_stack_names: Dict[str, str] = {}

        # Publisher for process crash notifications
        self._crash_publisher = self.create_publisher(
            String,
            "launch_plugin/process_crashed",
            10
        )

        # Ensure a valid asyncio loop exists to avoid 'There is no current event loop' warnings
        try:
            self.async_loop = asyncio.get_running_loop()
        except RuntimeError:
            self.async_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.async_loop)
        self.timer = self.create_timer(
            0.1, self.run_async_loop, callback_group=ReentrantCallbackGroup()
        )

        # Background process health monitor timer
        self._process_monitor_timer = self.create_timer(
            self.PROCESS_MONITOR_INTERVAL,
            self._monitor_processes,
            callback_group=ReentrantCallbackGroup()
        )

        atexit.register(self._cleanup_managed_launchers)


    def destroy_node(self) -> bool:
        """Ensure launched processes are cleaned up when the node is destroyed."""
        self.get_logger().info("Destroying launch_plugin node; terminating active launchers.")
        for launch_file in list(self._managed_launchers.keys()):
            self._terminate_launch_process(launch_file)
        return super().destroy_node()

    def run_async_loop(self):
        """Periodically step through the asyncio event loop."""
        self.async_loop.stop()
        self.async_loop.run_forever()
    

    def source_workspaces(self, current: StackManifest):
        """
        Source the specified workspaces and update the environment variables.
        """
        if not current:
            self.get_logger().error("No valid current stack available to source workspaces.")
            return

        source_data = current.source
        if not source_data:
            self.get_logger().debug("No source data in current stack.")
            return

        try:
            sources = json.loads(source_data) if isinstance(source_data, str) else source_data
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse source data as JSON.")
            return

        # Get stack name - check metadata.name first, then name, then default
        stack_name = self._get_stack_name(current)
        workspace_dir = os.path.join(
            WORKSPACES_PATH, stack_name.replace(" ", "_")
        )

        def source_script(name: str, script_path: str) -> None:
            self.get_logger().info(f"Sourcing: {name} | {script_path}")
            command = f"bash -c 'source {script_path} && env'"
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

        for name, script_path in sources.items():
            source_script(name, script_path)

        if not sources:
            default_install = os.path.join(workspace_dir, "install", "setup.bash")
            if os.path.exists(default_install):
                self.get_logger().info(
                    "No explicit source scripts provided; sourcing install/setup.bash"
                )
                source_script("workspace_install", default_install)
            else:
                self.get_logger().debug(
                    "No explicit source scripts provided and install/setup.bash not found; skipping sourcing."
                )

    def handle_start(
        self, request: LaunchPlugin.Request, response: LaunchPlugin.Response
    ):
        """
        Service handler for starting the stack using double dispatch pattern.

        Args:
            request (LaunchPlugin.Request): The service request.
            response (LaunchPlugin.Response): The service response.

        Returns:
            LaunchPlugin.Response: The response indicating success or failure.
        """
        handler, context = self.find_stack_handler(request)
        try:
            if handler and context:
                context.operation = StackOperation.START
                self.get_logger().info(
                    f"Start requested; current number of launched stacks={len(self._managed_launchers)}"
                )
                handler.apply_to_plugin(self, context, request, response)
                response.success = True
            else:
                response.success = False
                response.err_msg = "No current stack available or start flag not set."
                self.get_logger().info("No current stack available or start flag not set.")
        except Exception as e:
            self.get_logger().error(f"Exception occurred during start: {e}")
            response.err_msg = str(e)
            response.success = False

        response.output.current = request.input.current
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
        Service handler for killing the stack using double dispatch pattern.

        Args:
            request (LaunchPlugin.Request): The service request.
            response (LaunchPlugin.Response): The service response.

        Returns:
            LaunchPlugin.Response: The response indicating success or failure.
        """
        try:
            self.get_logger().info(
                f"Kill requested; current number of launched stacks={len(self._managed_launchers)}"
            )

            # Check if this is a kill-only payload (just stackId reference, not a full manifest)
            is_kill_only_payload = False
            stack_id = None
            if request.input.current.stack:
                try:
                    stack_data = json.loads(request.input.current.stack)
                    # Kill payloads have stackId in value key or path ending in /kill
                    stack_id = stack_data.get("value", {}).get("stackId") or stack_data.get("stackId")
                    if stack_id or stack_data.get("path", "").endswith("/kill"):
                        is_kill_only_payload = True
                except json.JSONDecodeError:
                    pass

            if is_kill_only_payload:
                # Kill all managed launchers for this stack
                self.get_logger().info(f"Kill action for stack_id={stack_id}")
                killed_count = 0
                for launch_file in list(self._managed_launchers.keys()):
                    self._terminate_launch_process(launch_file)
                    killed_count += 1
                self.get_logger().info(f"Killed {killed_count} launcher(s)")

                # Update the current stack in the twin service
                if stack_id:
                    self._set_current_stack(stack_id, state="killed")

                response.success = True
                response.output.current = request.input.current
                return response

            # Try to use handler for full stack manifests
            handler, context = self.find_stack_handler(request)
            if handler and context:
                context.operation = StackOperation.KILL
                success = handler.apply_to_plugin(self, context, request=request, response=response)
                response.success = True
            else:
                response.success = False
                response.err_msg = "No current stack available or start flag not set."
                self.get_logger().warning("No current stack available or start flag not set in kill request.")
        except Exception as e:
            self.get_logger().error(f"Exception occurred during kill: {e}")
            response.err_msg = str(e)
            response.success = False

        response.output.current = request.input.current
        return response

    def handle_apply(
        self, request: LaunchPlugin.Request, response: LaunchPlugin.Response
    ):
        """
        Service handler for applying the stack configuration using double dispatch pattern.

        Args:
            request (LaunchPlugin.Request): The service request.
            response (LaunchPlugin.Response): The service response.

        Returns:
            LaunchPlugin.Response: The response indicating success or failure.
        """
        handler, context = self.find_stack_handler(request)
        try:
            if handler and context:
                context.operation = StackOperation.APPLY
                self.get_logger().info(
                    f"Apply requested; current number of launched stacks={len(self._managed_launchers)}"
                )
                success = handler.apply_to_plugin(self, context, request, response)
                response.success = True
            else:
                response.success = False
                response.err_msg = "No current stack available or start flag not set."
                self.get_logger().warning("No current stack available or start flag not set in kill request.")
        except Exception as e:
            self.get_logger().error(f"Exception occurred during kill: {e}")
            response.err_msg = str(e)
            response.success = False
            
        response.output.current = request.input.current
        return response
    

    def _launch_via_ros2(self, context: StackContext, launch_file: str) -> bool:
        """
        Launch the given file using appropriate method based on file type.

        Supports:
        - ROS 2 launch files (.launch.py, .launch.xml, .launch.yaml) via Ros2LaunchParent
        - Shell scripts (.sh) via subprocess
        """
        self._terminate_launch_process(launch_file)

        full_launch_file = self.find_file(context.workspace_path, launch_file)

        if not full_launch_file or not os.path.exists(full_launch_file):
            self.get_logger().error(f"Launch file not found: {full_launch_file}")
            return False

        # Determine launch method based on file extension
        if full_launch_file.endswith('.sh'):
            return self._launch_via_shell(context, full_launch_file, launch_file)
        else:
            return self._launch_via_ros2_launch(context, full_launch_file, launch_file)

    def _launch_via_shell(self, context: StackContext, full_path: str, launch_file: str) -> bool:
        """
        Launch a shell script as a subprocess with background health monitoring.

        The process is started immediately and monitored by a background timer.
        If the process crashes, a notification is published to the crash topic.

        Args:
            context: Stack context with workspace information
            full_path: Full path to the shell script
            launch_file: Original launch file name (for tracking)

        Returns:
            True if process started successfully, False otherwise
        """
        self.get_logger().info(f"Starting shell script: {full_path}")

        # Source workspace setup.bash if available
        workspace_dir = context.workspace_path
        setup_bash = os.path.join(workspace_dir, "install", "setup.bash")

        try:
            # Build the command - source setup.bash if available, then run script
            if os.path.exists(setup_bash):
                command = f"source {setup_bash} && bash {full_path}"
            else:
                command = f"bash {full_path}"

            # Start the process in the background
            process = subprocess.Popen(
                command,
                shell=True,
                executable="/bin/bash",
                cwd=workspace_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid  # Create new process group for cleanup
            )

            # Store process for lifecycle management (using a wrapper object)
            wrapper = ShellProcessWrapper(process, full_path)
            self._managed_launchers[launch_file] = wrapper

            # Track stack name for crash reporting
            stack_name = getattr(context, 'stack_name', None) or os.path.basename(workspace_dir)
            self._launcher_stack_names[launch_file] = stack_name

            self.get_logger().info(
                f"Shell script launched with PID {process.pid}: {launch_file} (stack: {stack_name})"
            )
            self.get_logger().info("Process will be monitored by background watchdog")
            return True

        except Exception as exc:
            self.get_logger().error(f"Failed to start shell script: {exc}")
            raise RuntimeError(f"Failed to start shell script: {exc}")

    def _launch_via_ros2_launch(self, context: StackContext, full_path: str, launch_file: str) -> bool:
        """
        Launch a ROS 2 launch file using Ros2LaunchParent.

        Args:
            context: Stack context with workspace information
            full_path: Full path to the launch file
            launch_file: Original launch file name (for tracking)
        """
        # Source the workspace before launching
        working_dir = os.path.dirname(os.path.dirname(full_path))
        setup_bash = os.path.join(working_dir, "install", "setup.bash")
        if os.path.exists(setup_bash):
            self._source_workspace(setup_bash)

        self.get_logger().info(f"Starting launch via Ros2LaunchParent: {full_path}")

        try:
            # Create a Ros2LaunchParent instance with empty args
            launcher = Ros2LaunchParent([])

            # Use the async method to launch the file
            async def _do_launch():
                await launcher.launch_a_launch_file(
                    launch_file_path=full_path,
                    launch_file_arguments=[],
                    noninteractive=True
                )

            # Schedule the launch on the async loop
            asyncio.run_coroutine_threadsafe(_do_launch(), self.async_loop)

            # Track the launcher for lifecycle management
            self._managed_launchers[launch_file] = launcher
            self.get_logger().info(f"Launch initiated for {launch_file}")
            return True

        except Exception as exc:
            self.get_logger().error(f"Failed to start launch via Ros2LaunchParent: {exc}")
            raise RuntimeError(f"Failed to start launch process: {exc}")

    def _source_workspace(self, setup_bash: str) -> None:
        """Source a workspace setup.bash file to update environment."""
        try:
            command = f"bash -c 'source {setup_bash} && env'"
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                shell=True,
                executable="/bin/bash",
                check=True,
                text=True,
            )
            env_vars = dict(
                line.split("=", 1)
                for line in result.stdout.splitlines()
                if "=" in line
            )
            os.environ.update(env_vars)
            self.get_logger().debug(f"Sourced workspace: {setup_bash}")
        except Exception as e:
            self.get_logger().warning(f"Failed to source workspace {setup_bash}: {e}")

    def _set_current_stack(self, stack_id: str, state: str = "unknown") -> bool:
        """
        Update the current stack in the CoreTwin service.

        Args:
            stack_id: The stack ID to set as current
            state: The state of the stack (e.g., "running", "killed", "unknown")

        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.set_stack_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning("CoreTwin set_current_stack service not available")
                return False

            request = CoreTwin.Request()
            request.input = stack_id

            future = self.set_stack_cli.call_async(request)
            self.get_logger().info(f"Setting current stack to {stack_id} with state={state}")
            return True

        except Exception as e:
            self.get_logger().error(f"Error calling set_current_stack: {e}")
            return False

    def _terminate_launch_process(self, launch_file: str) -> None:
        """
        Terminate a launch process using Ros2LaunchParent.kill().

        Uses Ros2LaunchParent's built-in lifecycle management for consistent termination.
        """
        if launch_file in self._managed_launchers:
            launcher = self._managed_launchers[launch_file]
            self.get_logger().info(f"Terminating launcher for {launch_file}")
            try:
                launcher.kill()
                self.get_logger().info(f"Launcher for {launch_file} terminated.")
            except Exception as e:
                self.get_logger().warning(f"Error terminating launcher for {launch_file}: {e}")
            finally:
                self._managed_launchers.pop(launch_file, None)
                self._launcher_stack_names.pop(launch_file, None)

    def _cleanup_managed_launchers(self) -> None:
        """Clean up all managed launchers on exit."""
        for launch_file in list(self._managed_launchers.keys()):
            try:
                launcher = self._managed_launchers[launch_file]
                launcher.kill()
            except Exception as e:
                self.get_logger().warning(f"Error cleaning up launcher {launch_file}: {e}")
        self._managed_launchers.clear()
        self._launcher_stack_names.clear()

    def _monitor_processes(self) -> None:
        """
        Background timer callback to monitor managed processes for crashes.

        Checks all managed shell processes and publishes a crash notification
        if any have exited unexpectedly.
        """
        for launch_file in list(self._managed_launchers.keys()):
            launcher = self._managed_launchers.get(launch_file)
            if launcher is None:
                continue

            # Only check ShellProcessWrapper instances (shell scripts)
            if isinstance(launcher, ShellProcessWrapper):
                exit_code = launcher.process.poll()
                if exit_code is not None:
                    # Process has exited - check if it was an unexpected crash
                    stack_name = self._launcher_stack_names.get(launch_file, "unknown")

                    # Try to capture any output for debugging
                    stdout_output = ""
                    try:
                        if launcher.process.stdout:
                            stdout_output = launcher.process.stdout.read().decode('utf-8', errors='replace')
                    except Exception:
                        pass

                    self.get_logger().error(
                        f"Process {launch_file} crashed unexpectedly (exit code {exit_code})"
                    )

                    # Publish crash notification
                    self._publish_crash_notification(
                        process_name=launch_file,
                        exit_code=exit_code,
                        stack_name=stack_name,
                        error_message=f"Process exited with code {exit_code}",
                        process_output=stdout_output[-500:] if stdout_output else ""
                    )

                    # Remove from managed launchers
                    self._managed_launchers.pop(launch_file, None)
                    self._launcher_stack_names.pop(launch_file, None)

    def _publish_crash_notification(
        self,
        process_name: str,
        exit_code: int,
        stack_name: str,
        error_message: str,
        process_output: str = ""
    ) -> None:
        """Publish a process crash notification to the crash topic."""
        crash_data = {
            "process_name": process_name,
            "exit_code": exit_code,
            "stack_name": stack_name,
            "error_message": error_message,
            "process_output": process_output
        }

        msg = String()
        msg.data = json.dumps(crash_data)
        self._crash_publisher.publish(msg)

        self.get_logger().info(
            f"Published crash notification for {process_name} (stack: {stack_name})"
        )


def main():
    rclpy.init()
    launch_plugin = MutoDefaultLaunchPlugin()
    rclpy.spin(launch_plugin)
    launch_plugin.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
