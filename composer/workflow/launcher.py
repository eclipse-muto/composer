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
import signal
import launch
import multiprocessing
import concurrent.futures
import rclpy.logging
from collections import OrderedDict
from typing import (
    List,
    Text,
    Tuple,
    Dict
)
from launch import LaunchDescription, LaunchService
import asyncio
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from composer.introspection.model.difference import Difference


class Ros2LaunchParent:
    """
    Manages the launching of ROS2 nodes in separate processes.
    """

    def __init__(self, launch_arguments):
        # A multiprocessing manager so we can keep an active list of nodes safely across processes
        self.manager = multiprocessing.Manager()
        self._active_nodes = self.manager.list()
        self._lock = self.manager.Lock()
        self._stop_event = None
        self._process = None
        self.launch_arguments = launch_arguments

    def __del__(self):
        # Cleanly shut down the multiprocessing manager
        self.manager.shutdown()

    def parse_launch_arguments(self, launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
        """
        Parse the given launch arguments from the command line into a list of (key, value) pairs.

        Example argument: "robot_name:=my_robot"
        Returns: [("robot_name", "my_robot")]
        """
        parsed_launch_arguments = OrderedDict()
        for argument in launch_arguments:
            count = argument.count(':=')
            if count == 0 or argument.startswith(':=') or (count == 1 and argument.endswith(':=')):
                raise RuntimeError(
                    f"malformed launch argument '{argument}', expected format '<name>:=<value>'"
                )
            name, value = argument.split(':=', maxsplit=1)
            # If the same argument name appears multiple times, last one wins
            parsed_launch_arguments[name] = value
        return parsed_launch_arguments.items()

    def start(self, launch_description: LaunchDescription):
        """
        Starts the launch process for the given launch description in a separate process.
        This method is for the old-style-stack.
        """
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=self._run_process, args=(self._stop_event, launch_description), daemon=True)
        self._process.start()

    def _run_process(self, stop_event, launch_description):
        """
        Callable for the multiprocessing.Process to run the launch service in a separate process.
        The target method for the process, running the launch service with event handling.
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        is_mock_loop = not isinstance(loop, asyncio.AbstractEventLoop)

        launch_description.add_action(
            RegisterEventHandler(
                OnProcessStart(
                    on_start=lambda event, context: self._event_handler(
                        'start', event, self._active_nodes, self._lock
                    )
                )
            )
        )
        launch_description.add_action(
            RegisterEventHandler(
                OnProcessExit(
                    on_exit=lambda event, context: self._event_handler(
                        'exit', event, self._active_nodes, self._lock
                    )
                )
            )
        )

        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(launch_description)
        run_coro = launch_service.run_async()

        # In unit tests, asyncio.new_event_loop may be patched to a mock.
        # If so, avoid creating/awaiting real coroutines to prevent warnings.
        if is_mock_loop:
            try:
                # Close the coroutine to prevent "never awaited" warnings
                run_coro.close()
            except Exception:
                pass
            return

        launch_task = loop.create_task(run_coro)

        async def wait_for_stop_event():
            while not stop_event.is_set():
                await asyncio.sleep(0.1)
            launch_service.shutdown()

        try:
            loop.run_until_complete(asyncio.gather(launch_task, wait_for_stop_event()))
        except Exception as e:
            print(f"An exception occurred during the launch process: {e}")
        finally:
            loop.close()


    async def launch_a_launch_file(
        self,
        launch_file_path: str,
        launch_file_arguments: List[str],
        noninteractive: bool = False,
        debug: bool = False,
        dry_run: bool = False
    ):
        """
        Launch a given launch file (by path) and pass it the given launch file arguments.
        This method is non-blocking and runs the launch file asynchronously.

        Args:
            launch_file_path (str): Path to the .launch.py file to run
            launch_file_arguments (List[str]): e.g. ["robot_name:=my_robot"]
            noninteractive (bool): If True, run launch in noninteractive mode
            debug (bool): If True, run launch in debug mode
            dry_run (bool): If True, don't actually run the launch, just parse
        """
        rclpy.logging.get_logger("muto_launch_parent").info(
            f"Launching file: {launch_file_path} with arguments: {launch_file_arguments}"
        )

        launch_service = launch.LaunchService(
            argv=launch_file_arguments,
            noninteractive=noninteractive,
            debug=debug
        )

        parsed_launch_arguments = self.parse_launch_arguments(launch_file_arguments)

        launch_description = launch.LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    launch_file_path
                ),
                launch_arguments=parsed_launch_arguments,
            ),
        ])

        launch_description.add_action(
            RegisterEventHandler(
                OnProcessStart(
                    on_start=lambda event, context: self._event_handler(
                        'start', event, self._active_nodes, self._lock
                    )
                )
            )
        )
        launch_description.add_action(
            RegisterEventHandler(
                OnProcessExit(
                    on_exit=lambda event, context: self._event_handler(
                        'exit', event, self._active_nodes, self._lock
                    )
                )
            )
        )

        launch_service.include_launch_description(launch_description)

        if not dry_run:
            # Using run_async() rather than run() to make it non-blocking
            await launch_service.run_async(shutdown_when_idle=False)

        return launch_description, launch_service

    async def launch_a_launch_description(
        self,
        launch_description: launch.LaunchDescription,
        launch_file_arguments: List[str] = None,
        noninteractive: bool = False,
        debug: bool = False,
        dry_run: bool = False
    ):
        """
        Launch a given LaunchDescription with optional arguments.
        This is an alternative to launch_a_launch_file(...) if you already have a LaunchDescription in memory.

        Args:
            launch_description (LaunchDescription): The LaunchDescription object to run
            launch_file_arguments (List[str]): e.g. ["robot_name:=my_robot"]
            noninteractive (bool): If True, run launch in noninteractive mode
            debug (bool): If True, run launch in debug mode
            dry_run (bool): If True, don't actually run the launch, just parse
        """
        if launch_file_arguments is None:
            launch_file_arguments = []

        rclpy.logging.get_logger("muto_launch_parent").info(
            f"Launching LaunchDescription with arguments: {launch_file_arguments}"
        )

        launch_service = launch.LaunchService(
            argv=launch_file_arguments,
            noninteractive=noninteractive,
            debug=debug
        )

        # Register the same event handlers so we can track node processes
        launch_description.add_action(
            RegisterEventHandler(
                OnProcessStart(
                    on_start=lambda event, context: self._event_handler(
                        'start', event, self._active_nodes, self._lock
                    )
                )
            )
        )
        launch_description.add_action(
            RegisterEventHandler(
                OnProcessExit(
                    on_exit=lambda event, context: self._event_handler(
                        'exit', event, self._active_nodes, self._lock
                    )
                )
            )
        )

        launch_service.include_launch_description(launch_description)

        if not dry_run:
            await launch_service.run_async(shutdown_when_idle=False)

        return launch_description, launch_service

    def shutdown(self):
        """
        Signals the launch process to shut down and waits for it to terminate.
        """
        if self._stop_event:
            self._stop_event.set()
        if self._process is not None:
            self._process.join(timeout=20.0)
            if self._process.is_alive():
                self._process.terminate()
                rclpy.logging.get_logger("muto_launch_parent").info(
                    "The process did not terminate gracefully and was terminated forcefully."
                )

    def kill(self):
        """
        Kills all active nodes by sending SIGKILL directly to ensure immediate termination.
        Shuts down the LaunchService after killing the nodes.
        """
        with self._lock:
            if not self._active_nodes:
                rclpy.logging.get_logger("muto_launch_parent").info("No active nodes to kill.")
                return

            def kill_node(node):
                for process_name, pid in node.items():
                    try:
                        os.kill(pid, signal.SIGKILL)
                        rclpy.logging.get_logger("muto_launch_parent").info(
                            f"Sent SIGKILL to process {process_name} (PID {pid})"
                        )
                    except ProcessLookupError:
                        rclpy.logging.get_logger("muto_launch_parent").info(
                            f"Process {process_name} (PID {pid}) already terminated."
                        )
                    except Exception as e:
                        rclpy.logging.get_logger("muto_launch_parent").info(
                            f"Failed to kill process {process_name} (PID {pid}): {e}"
                        )

            # Kill everything in parallel using a ThreadPoolExecutor
            with concurrent.futures.ThreadPoolExecutor() as executor:
                executor.map(kill_node, self._active_nodes)

            # Clear out the active node list
            self._active_nodes[:] = []

        if self._process:
            rclpy.logging.get_logger("muto_launch_parent").info("Shutting down the launch service")
            if self._stop_event:
                self._stop_event.set()
            self._process.join(timeout=10.0)
            if self._process.is_alive():
                rclpy.logging.get_logger("muto_launch_parent").warn(
                    "Launch service did not stop gracefully, terminating forcefully."
                )
                self._process.terminate()

    def kill_nodes_by_name(self, node_names: List[str]):
        """
        Kills only the active nodes whose process_name is in node_names.
        If your node names differ from the process_name assigned by ROS 2,
        you may need to store & kill by PID or adapt this method.
        """
        with self._lock:
            if not self._active_nodes:
                rclpy.logging.get_logger("muto_launch_parent").info("No active nodes to kill.")
                return

            def kill_node_if_needed(node):
                for process_name, pid in node.items():
                    if process_name in node_names:
                        try:
                            os.kill(pid, signal.SIGKILL)
                            rclpy.logging.get_logger("muto_launch_parent").info(
                                f"Sent SIGKILL to process {process_name} (PID {pid})"
                            )
                        except ProcessLookupError:
                            rclpy.logging.get_logger("muto_launch_parent").info(
                                f"Process {process_name} (PID {pid}) already terminated."
                            )
                        except Exception as e:
                            rclpy.logging.get_logger("muto_launch_parent").error(
                                f"Failed to kill process {process_name} (PID {pid}): {e}"
                            )

            updated_nodes = []
            with concurrent.futures.ThreadPoolExecutor() as executor:
                for node in self._active_nodes:
                    executor.submit(kill_node_if_needed, node)
                    # If the node references a name we killed, we skip it
                    keep_this = True
                    for process_name, _pid in node.items():
                        if process_name in node_names:
                            keep_this = False
                    if keep_this:
                        updated_nodes.append(node)

            self._active_nodes[:] = updated_nodes

        # Optionally, if no nodes remain, shut down the service
        if not self._active_nodes and self._process:
            rclpy.logging.get_logger("muto_launch_parent").info("All requested nodes killed, shutting down the launch service.")
            if self._stop_event:
                self._stop_event.set()
            self._process.join(timeout=10.0)
            if self._process.is_alive():
                rclpy.logging.get_logger("muto_launch_parent").warn(
                    "Launch service did not stop gracefully, terminating forcefully."
                )
                self._process.terminate()

    def _event_handler(self, action, event, nodes_list, lock):
        """
        Generic event handler for both process start and exit events.
        Maintains a list of active nodes as a shared list of dicts: [{process_name: pid}, ...]
        """
        with lock:
            if action == 'start':
                rclpy.logging.get_logger("muto_launch_parent").info(
                    f"Node started: {event.process_name} with PID {event.pid}"
                )
                nodes_list.append({event.process_name: event.pid})
            elif action == 'exit':
                rclpy.logging.get_logger("muto_launch_parent").info(
                    f"Node exited: {event.process_name} with PID {event.pid}"
                )
                nodes_list[:] = [
                    node for node in nodes_list
                    if node.get(event.process_name) != event.pid
                ]

        rclpy.logging.get_logger("muto_launch_parent").info(
            f"Active nodes after {action}: {nodes_list}"
        )
        if not nodes_list and action == 'exit':
            self.shutdown()

    def create_launch_description_for_added_nodes(
        self, added_nodes: Dict[str, Dict[str, str]]
    ) -> launch.LaunchDescription:
        """
        Given 'added_nodes' from Delta, build a LaunchDescription with all the new Node actions.
        Each node entry is typically:
          {
            "name": "some_node",
            "namespace": "/",
            "package": "pkg_name",
            "executable": "exe_name",
            ...
          }
        """
        ld = launch.LaunchDescription()
        for key, node_info in added_nodes.items():
            node_name = node_info.get("name", "unnamed_node")
            node_ns = node_info.get("namespace", "/")
            pkg = node_info.get("package", "")
            exe = node_info.get("executable", "")

            if node_ns == "/":
                node_ns = ""

            node_action = Node(
                package=pkg,
                executable=exe,
                name=node_name,
                namespace=node_ns,
                output="screen",
            )
            ld.add_action(node_action)

        return ld

    def apply_delta(self, diff_result: Difference, extra_args: List[str], async_loop: asyncio.AbstractEventLoop):
        """
        Orchestrates:
          - Launching all 'added' nodes
          - Killing 'removed' nodes
          - Doing nothing for 'common' nodes
        Based on the Difference object from Delta.

        Args:
            diff_result (Difference): The difference containing added_nodes, removed_nodes, common_nodes
            extra_args (List[str]): e.g. ["foo:=bar"] to pass to the launch service
            async_loop (AbstractEventLoop): The asyncio loop in which to schedule the launch
        """
        # 1) Launch 'added' nodes
        if diff_result.added_nodes:
            ld = self.create_launch_description_for_added_nodes(diff_result.added_nodes)

            async def _launch():
                await self.launch_a_launch_description(
                    ld,
                    launch_file_arguments=extra_args,
                    noninteractive=True
                )

            asyncio.run_coroutine_threadsafe(_launch(), async_loop)

        # 2) Kill 'removed' nodes
        if diff_result.removed_nodes:
            removed_names = []
            for key, node_info in diff_result.removed_nodes.items():
                removed_names.append(node_info["name"])

            if removed_names:
                rclpy.logging.get_logger("muto_launch_parent").info(
                    f"Killing removed nodes: {removed_names}"
                )
                self.kill_nodes_by_name(removed_names)

        # 3) 'common_nodes' remain untouched
        rclpy.logging.get_logger("muto_launch_parent").info(
            "Done applying delta changes (added, removed, left common)."
        )
