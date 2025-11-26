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
import threading
import signal
import atexit
from typing import Optional, Set, Dict
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from muto_msgs.msg import StackManifest
from muto_msgs.srv import LaunchPlugin, CoreTwin
from composer.workflow.launcher import Ros2LaunchParent
from composer.plugins.provision_plugin import WORKSPACES_PATH
from composer.utils.stack_parser import StackParser
from .base_plugin import BasePlugin, StackContext, StackOperation


class MutoDefaultLaunchPlugin(BasePlugin):
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

        # self.create_subscription(StackManifest, "composed_stack", self.get_stack, 10)

        self.set_stack_cli = self.create_client(CoreTwin, "core_twin/set_current_stack")

        self.stack_parser = StackParser(self.get_logger())
        
        # A dictionary to keep track of managed subprocesses by launch file
        # initialize to empty dict
        self._managed_processes: Dict[str, subprocess.Popen] = dict()

        # Ensure a valid asyncio loop exists to avoid 'There is no current event loop' warnings
        try:
            self.async_loop = asyncio.get_running_loop()
        except RuntimeError:
            self.async_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.async_loop)
        self.timer = self.create_timer(
            0.1, self.run_async_loop, callback_group=ReentrantCallbackGroup()
        )

        atexit.register(self._cleanup_managed_processes)


    def destroy_node(self) -> bool:
        """Ensure launched processes are cleaned up when the node is destroyed."""
        self.get_logger().info("Destroying launch_plugin node; terminating active launch process if any.")
        for launch_file in list(self._managed_processes.keys()):
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
                    f"Start requested; current number of launched stacks={len(self._managed_processes)}"
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
        handler, context = self.find_stack_handler(request)
        try:
            if handler and context:
                context.operation = StackOperation.KILL
                self.get_logger().info(
                    f"Kill requested; current number of launched stacks={len(self._managed_processes)}"
                )
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
                    f"Apply requested; current number of launched stacks={len(self._managed_processes)}"
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
    

    def _launch_via_ros2(self, context: StackContext, launch_file: str) -> None:
        """Launch the given launch file in a subprocess using ros2 launch."""
        self._terminate_launch_process(launch_file)

            
        full_launch_file = self.find_file(context.workspace_path, launch_file)
        
        if not os.path.exists(full_launch_file):
            if self.logger:
                self.logger.error(f"Launch file not found: {full_launch_file}")
            return False
            
            
        # Create a shell script to source the environment if needed
        # and launch with the additional launch arguments
        launchCommand = ["ros2", "launch", full_launch_file]
        #if self.launch_arguments:
        #    launchCommand.extend(self.launch_arguments)
        
        workingDir = os.path.dirname(os.path.dirname(full_launch_file))
        script_path = os.path.join(workingDir, "launch_script.sh")
        script_content = f"#!/bin/bash\nsource {workingDir}/install/setup.bash\nexec \"$@\""
        with open(script_path, "w") as script_file:
            script_file.write(script_content)
        os.chmod(script_path, 0o755)
        
        # Use the shell script to launch with proper environment sourcing
        command = [script_path] + launchCommand

        env = os.environ.copy()
        self.get_logger().info(f"Launch PATH: {env.get('PATH', '')}")
        self.get_logger().info(
            f"Starting launch process in {workingDir}: {' '.join(command)}"
        )
        try:
            launch_process = subprocess.Popen(
                command,
                env=env,
                cwd=workingDir or None,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid,
            )
            self._managed_processes[launch_file] = launch_process
            self.get_logger().info(
                f"Launch process started with PID {launch_process.pid}"
            )
            if launch_process.stdout:
                threading.Thread(
                    target=self._log_stream,
                    args=(launch_process.stdout, "stdout"),
                    daemon=True,
                ).start()
            if launch_process.stderr:
                threading.Thread(
                    target=self._log_stream,
                    args=(launch_process.stderr, "stderr"),
                    daemon=True,
                ).start()
            threading.Thread(
                target=self._monitor_process,
                daemon=True,
            ).start()
        except FileNotFoundError:
            raise FileNotFoundError(
                "ros2 command not found in PATH while launching stack"
            )
        except Exception as exc:
            raise RuntimeError(f"Failed to start launch process: {exc}")

    def _terminate_launch_process(self, launch_file: str) -> None:
        if launch_file in self._managed_processes:
            launch_process = self._managed_processes[launch_file]
            if launch_process.poll() is None:
                self.get_logger().info("Terminating existing launch process")
                try:
                    pgid = os.getpgid(launch_process.pid)
                    self.get_logger().debug(f"Sending SIGTERM to process group {pgid}")
                    os.killpg(pgid, signal.SIGTERM)
                except ProcessLookupError:
                    self.get_logger().debug("Process group already gone when sending SIGTERM")
                pass
            try:
                launch_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    pgid = os.getpgid(launch_process.pid)
                    self.get_logger().warning(f"Process did not exit; sending SIGKILL to {pgid}")
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    self.get_logger().debug("Process group already gone when sending SIGKILL")
                self.get_logger().warning(
                    "Launch process did not terminate gracefully; killed."
                )
            finally:
                self.get_logger().info("Existing launch process terminated.")
            self._managed_processes.pop(launch_file, None)

    def _log_stream(self, stream, label: str) -> None:
        for line in iter(stream.readline, ""):
            text = line.strip()
            if text:
                self.get_logger().info(f"launch {label}: {text}")
        stream.close()

    def _monitor_process(self) -> None:
        if not self.launch_process:
            return
        process = self.launch_process
        returncode = process.wait()
        self._managed_processes.discard(process)
        if process is self.launch_process:
            self.launch_process = None
        self.get_logger().info(
            f"Launch process exited with return code {returncode}"
        )

    def _cleanup_managed_processes(self) -> None:
        for process in list(self._managed_processes):
            if process.poll() is None:
                try:
                    pgid = os.getpgid(process.pid)
                    os.killpg(pgid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self._managed_processes.clear()



def main():
    rclpy.init()
    launch_plugin = MutoDefaultLaunchPlugin()
    rclpy.spin(launch_plugin)
    launch_plugin.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
