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
from typing import Optional, Set
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from muto_msgs.msg import StackManifest
from muto_msgs.srv import LaunchPlugin, CoreTwin
from composer.workflow.launcher import Ros2LaunchParent
from composer.plugins.provision_plugin import WORKSPACES_PATH
from composer.model.stack import Stack
from composer.utils.stack_parser import StackParser

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

        self.create_subscription(StackManifest, "composed_stack", self.get_stack, 10)

        self.set_stack_cli = self.create_client(CoreTwin, "core_twin/set_current_stack")

        self.current_stack: Optional[StackManifest] = None
        self.launch_arguments = []
        self.launcher = Ros2LaunchParent(self.launch_arguments)
        self.stack_parser = StackParser(self.get_logger())
        self.launch_process: Optional[subprocess.Popen] = None
        self._managed_processes: Set[subprocess.Popen] = set()

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
        self._terminate_launch_process()
        return super().destroy_node()

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
            self.get_logger().info(
                f"Launch arguments parsed: {self.launch_arguments}"
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing launch arguments: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def _get_payload_type_and_data(self, payload):
        """
        Determine the payload type and extract relevant data using the stack parser.
        
        Returns:
            tuple: (payload_type, stack_data, launch_file, command)
            payload_type: 'stack/json', 'stack/archive', or 'raw'
        """
        if not isinstance(payload, dict):
            return None, None, None, None
            
        # Use the stack parser to determine payload type and extract data
        parsed_payload = self.stack_parser.parse_payload(payload)
        if not parsed_payload:
            return None, None, None, None
            
        # Determine type based on content_type or structure
        content_type = parsed_payload.get("content_type")
        if content_type == "stack/archive":
            launch_file = parsed_payload.get("launch_file")
            command = parsed_payload.get("command", "launch")
            return "stack/archive", parsed_payload, launch_file, command
        elif content_type == "stack/json":
            # For stack/json, the parsed_payload is the launch data
            return "stack/json", parsed_payload, None, None
        elif parsed_payload.get("node") or parsed_payload.get("composable"):
            # Raw stack with node/composable
            return "raw", parsed_payload, None, None
        else:
            return None, None, None, None

    def _handle_stack_json_start(self, stack_data):
        """Handle start for stack/json payload type."""
        if stack_data:
            stack = Stack(manifest=stack_data)
            stack.launch(self.launcher)
            return True
        return False

    def _handle_raw_stack_start(self, stack_data):
        """Handle start for raw stack payload type."""
        if stack_data:
            stack = Stack(manifest=stack_data)
            stack.launch(self.launcher)
            return True
        return False

    def _handle_archive_start(self, launch_file):
        """Handle start for stack/archive payload type."""
        if launch_file:
            self._terminate_launch_process()
            workspace_path = os.path.join(
                WORKSPACES_PATH,
                self.current_stack.name.replace(" ", "_"),
            )
            full_launch_file = self.find_file(workspace_path, launch_file)
            if not full_launch_file:
                raise FileNotFoundError(f"Launch file not found: {launch_file}")
            self._launch_via_ros2(full_launch_file)
            return True
        return False

    def _handle_archive_kill(self, launch_file):
        """Handle kill for stack/archive payload type."""
        if launch_file:
            self._terminate_launch_process()
            self.get_logger().info("Launch process killed successfully.")
            return True
        return False

    def _handle_raw_stack_kill(self, stack_data):
        """Handle kill for raw stack payload type."""
        # For raw stacks, just terminate any running launch processes
        # self._terminate_launch_process()
        self.launcher.kill()

        self.get_logger().info("Launch process killed successfully.")
        return True

    def _handle_raw_stack_apply(self, stack_data):
        """Handle apply for raw stack payload type."""
        if stack_data:
            stack = Stack(manifest=stack_data)
            stack.apply(self.launcher)
            return True
        return False

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

        candidate = os.path.join(ws_path, file_name)
        if os.path.isfile(candidate):
            self.get_logger().info(f"Found file directly: {candidate}")
            return candidate

        basename = os.path.basename(file_name)
        for root, _, files in os.walk(ws_path):
            if basename in files:
                found_path = os.path.join(root, basename)
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
        Service handler for starting the stack.

        Args:
            request (LaunchPlugin.Request): The service request.
            response (LaunchPlugin.Response): The service response.

        Returns:
            LaunchPlugin.Response: The response indicating success or failure.
        """
        try:
            if request.start:
                if self.current_stack:
                    self.get_logger().info(
                        f"Start requested; current launch PID={getattr(self.launch_process, 'pid', None)}"
                    )
                    self.source_workspaces()
                    
                    # Parse payload and determine type
                    payload = {}
                    if self.current_stack.stack:
                        try:
                            payload = json.loads(self.current_stack.stack)
                        except (json.JSONDecodeError, TypeError):
                            # If stack is not valid JSON, treat as empty payload
                            payload = {}
                    payload_type, stack_data, launch_file, command = self._get_payload_type_and_data(payload)
                    
                    if payload_type == "stack/json":
                        success = self._handle_stack_json_start(stack_data)
                    elif payload_type == "stack/archive":
                        success = self._handle_archive_start(launch_file)
                    elif payload_type == "raw":
                        success = self._handle_raw_stack_start(stack_data)
                    else:
                        # Fallback to legacy script-based launch
                        success = self._handle_legacy_script_start()
                    
                    if success:
                        response.success = True
                        response.err_msg = ""
                    else:
                        response.success = False
                        response.err_msg = "No valid launch method found for the stack payload."
                        self.get_logger().warning("No valid launch method found for the stack payload.")
                else:
                    response.success = False
                    response.err_msg = "No default stack on device."
                    self.get_logger().info("No default stack on device.")
            else:
                response.success = False
                response.err_msg = "Start flag not set in request."
                self.get_logger().warning("Start flag not set in start request.")
        except Exception as e:
            self.get_logger().error(f"Exception occurred during start: {e}")
            response.err_msg = str(e)
            response.success = False
        return response

    def _handle_legacy_script_start(self):
        """Handle legacy script-based start for backward compatibility."""
        if self.current_stack.on_start and self.current_stack.on_kill:
            script = self.find_file(
                os.path.join(
                    WORKSPACES_PATH,
                    self.current_stack.name.replace(" ", "_"),
                ),
                self.current_stack.on_start,
            )
            if not script:
                raise FileNotFoundError(f"Script not found: {self.current_stack.on_start}")
            self.run_script(script)
            self.get_logger().info("Start script executed successfully.")
            return True
        elif self.current_stack.launch_description_source:
            # Legacy behavior: launch the description source directly
            launch_file = self.find_file(
                os.path.join(
                    WORKSPACES_PATH,
                    self.current_stack.name.replace(" ", "_"),
                ),
                self.current_stack.launch_description_source,
            )
            if not launch_file:
                raise FileNotFoundError(f"Launch file not found: {self.current_stack.launch_description_source}")
            self._launch_via_ros2(launch_file)
            return True
        return False

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
                    self.get_logger().info(
                        f"Kill requested; current launch PID={getattr(self.launch_process, 'pid', None)}"
                    )
                    
                    # Parse payload and determine type
                    payload = {}
                    if self.current_stack.stack:
                        try:
                            payload = json.loads(self.current_stack.stack)
                        except (json.JSONDecodeError, TypeError):
                            # If stack is not valid JSON, treat as empty payload
                            payload = {}
                    payload_type, stack_data, launch_file, command = self._get_payload_type_and_data(payload)
                    
                    if payload_type == "stack/archive":
                        success = self._handle_archive_kill(launch_file)
                    elif payload_type in ["stack/json", "raw"]:
                        success = self._handle_raw_stack_kill(stack_data)
                    else:
                        # Fallback: check for legacy on_kill script, otherwise terminate process
                        if self.current_stack.on_kill:
                            success = self._handle_legacy_script_kill()
                        else:
                            self._terminate_launch_process()
                            self.get_logger().info("Launch process killed successfully.")
                            success = True
                    
                    if success:
                        response.success = True
                        response.err_msg = "Handle kill success"
                    else:
                        response.success = False
                        response.err_msg = "No valid kill method found for the stack payload."
                        self.get_logger().warning("No valid kill method found for the stack payload.")
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

    def _handle_legacy_script_kill(self):
        """Handle legacy script-based kill for backward compatibility."""
        if self.current_stack.on_kill:
            script = self.find_file(
                os.path.join(
                    WORKSPACES_PATH,
                    self.current_stack.name.replace(" ", "_"),
                ),
                self.current_stack.on_kill,
            )
            if not script:
                raise FileNotFoundError(f"Script not found: {self.current_stack.on_kill}")
            self.run_script(script)
            self.get_logger().info("Kill script executed successfully.")
            return True
        return False

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
            if self.current_stack:
                # Parse payload and determine type
                payload = {}
                if self.current_stack.stack:
                    try:
                        payload = json.loads(self.current_stack.stack)
                    except (json.JSONDecodeError, TypeError):
                        # If stack is not valid JSON, treat as empty payload
                        payload = {}
                payload_type, stack_data, launch_file, command = self._get_payload_type_and_data(payload)
                
                stack_dict = None
                if payload_type == "stack/json":
                    stack_dict = stack_data
                elif payload_type == "raw":
                    stack_dict = stack_data
                elif payload_type == "stack/archive":
                    # For archive, we might need to apply the full payload or just the properties
                    stack_dict = payload
                else:
                    stack_dict = payload  # fallback
                
                if stack_dict:
                    self.get_logger().info(
                        f"Apply requested with stack manifest keys: {list(stack_dict.keys())}"
                    )
                    stack = Stack(manifest=stack_dict)
                    stack.apply(self.launcher)
                    response.success = True
                    response.err_msg = ""
                else:
                    response.success = False
                    response.err_msg = "No valid stack data found for apply operation."
            else:
                response.success = False
                response.err_msg = "No current stack available for apply operation."
        except Exception as e:
            self.get_logger().error(f"Exception occurred during apply: {e}")
            response.err_msg = str(e)
            response.success = False
        return response

    def _launch_via_ros2(self, launch_file: str) -> None:
        """Launch the given launch file in a subprocess using ros2 launch."""
        self._terminate_launch_process()

        command = ["ros2", "launch", launch_file]
        command.extend(self.launch_arguments)
        env = os.environ.copy()
        self.get_logger().info(f"Launch PATH: {env.get('PATH', '')}")
        self.get_logger().info(
            f"Starting launch process in {os.path.dirname(launch_file)}: {' '.join(command)}"
        )
        try:
            self.launch_process = subprocess.Popen(
                command,
                env=env,
                cwd=os.path.dirname(launch_file) or None,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid,
            )
            self._managed_processes.add(self.launch_process)
            self.get_logger().info(
                f"Launch process started with PID {self.launch_process.pid}"
            )
            if self.launch_process.stdout:
                threading.Thread(
                    target=self._log_stream,
                    args=(self.launch_process.stdout, "stdout"),
                    daemon=True,
                ).start()
            if self.launch_process.stderr:
                threading.Thread(
                    target=self._log_stream,
                    args=(self.launch_process.stderr, "stderr"),
                    daemon=True,
                ).start()
            threading.Thread(
                target=self._monitor_process,
                daemon=True,
            ).start()
        except FileNotFoundError:
            self.launch_process = None
            raise FileNotFoundError(
                "ros2 command not found in PATH while launching stack"
            )
        except Exception as exc:
            self.launch_process = None
            raise RuntimeError(f"Failed to start launch process: {exc}")

    def _terminate_launch_process(self) -> None:
        if self.launch_process and self.launch_process.poll() is None:
            self.get_logger().info("Terminating existing launch process")
            try:
                pgid = os.getpgid(self.launch_process.pid)
                self.get_logger().debug(f"Sending SIGTERM to process group {pgid}")
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                self.get_logger().debug("Process group already gone when sending SIGTERM")
                pass
            try:
                self.launch_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    pgid = os.getpgid(self.launch_process.pid)
                    self.get_logger().warning(f"Process did not exit; sending SIGKILL to {pgid}")
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    self.get_logger().debug("Process group already gone when sending SIGKILL")
                self.get_logger().warning(
                    "Launch process did not terminate gracefully; killed."
                )
            finally:
                self.get_logger().info("Existing launch process terminated.")
        self.launch_process = None

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
