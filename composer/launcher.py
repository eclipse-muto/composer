import os
import subprocess
import time
import signal
from collections import OrderedDict
import asyncio
import multiprocessing
from typing import (
    List,
    Text,
    Tuple
)
from launch import LaunchDescription, LaunchService
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
import launch
import concurrent.futures


class Ros2LaunchParent:
    """
    Manages the launching of ROS2 nodes in separate processes.
    """

    def __init__(self, launch_arguments):
        self.manager = multiprocessing.Manager()
        self._active_nodes = self.manager.list()
        self._lock = self.manager.Lock()
        self._stop_event = None
        self._process = None
        self.launch_arguments = launch_arguments

    def __del__(self):
        self.manager.shutdown()

    def parse_launch_arguments(self, launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
        """Parse the given launch arguments from the command line, into list of tuples for launch."""
        parsed_launch_arguments = OrderedDict()
        for argument in launch_arguments:
            count = argument.count(':=')
            if count == 0 or argument.startswith(':=') or (count == 1 and argument.endswith(':=')):
                raise RuntimeError(
                    f"malformed launch argument '{argument}', expected format '<name>:=<value>'")
            name, value = argument.split(':=', maxsplit=1)
            # last one wins is intentional
            parsed_launch_arguments[name] = value
        return parsed_launch_arguments.items()

    async def launch_a_launch_file(
        self,
        launch_file_path,
        launch_file_arguments,
        noninteractive=False,
        debug=False
    ):
        """
        Launch a given launch file (by path) and pass it the given launch file arguments.
        This method is non-blocking and runs the launch file asynchronously.
        """
        print(
            f"Launching file: {launch_file_path} with arguments: {launch_file_arguments}")

        launch_service = launch.LaunchService(
            argv=launch_file_arguments,
            noninteractive=noninteractive,
            debug=debug
        )

        parsed_launch_arguments = self.parse_launch_arguments(
            launch_file_arguments)

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

        # Using run_async() rather than run() to make it non-blocking
        await launch_service.run_async(shutdown_when_idle=False)
        return launch_description, launch_service

    def shutdown(self):
        """
        Signals the launch process to shut down and waits for it to terminate.
        """
        self._stop_event.set()
        self._process.join(timeout=20.0)
        if self._process.is_alive():
            self._process.terminate()
            print(
                "The process did not terminate gracefully and was terminated forcefully.")

    def kill(self):
        """
        Kills all active nodes by sending SIGKILL directly to ensure immediate termination.
        Shuts down the LaunchService after killing the nodes.
        """
        with self._lock:
            if not self._active_nodes:
                print("No active nodes to kill.")
                return

            def kill_node(node):
                for process_name, pid in node.items():
                    try:
                        os.kill(pid, signal.SIGKILL)
                        print(
                            f"Sent SIGKILL to process {process_name} (PID {pid})")
                    except ProcessLookupError:
                        print(
                            f"Process {process_name} (PID {pid}) already terminated.")
                    except Exception as e:
                        print(
                            f"Failed to kill process {process_name} (PID {pid}): {e}")

            with concurrent.futures.ThreadPoolExecutor() as executor:
                executor.map(kill_node, self._active_nodes)

            self._active_nodes[:] = []

        if self._process:
            print("Shutting down the launch service")
            self._stop_event.set()
            self._process.join(timeout=10.0)
            if self._process.is_alive():
                print("Launch service did not stop gracefully, terminating forcefully.")
                self._process.terminate()

    def _event_handler(self, action, event, nodes_list, lock):
        """
        Generic event handler for both process start and exit events.
        """
        with lock:
            if action == 'start':
                print(
                    f"Node started: {event.process_name} with PID {event.pid}")
                nodes_list.append({event.process_name: event.pid})
            elif action == 'exit':
                print(
                    f"Node exited: {event.process_name} with PID {event.pid}")
                nodes_list[:] = [node for node in nodes_list if node.get(
                    event.process_name) != event.pid]

        print(f"Active nodes after {action}: {nodes_list}")
        if not nodes_list and action == 'exit':
            self.shutdown()

