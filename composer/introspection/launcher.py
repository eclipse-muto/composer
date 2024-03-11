#
#  Copyright (c) 2024 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#

import asyncio
import multiprocessing
from launch import LaunchDescription, LaunchService
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import FindExecutable

class Ros2LaunchParent:
    """
    Manages the launching of ROS2 nodes in a separate process and monitors their lifecycle events.
    """
    
    def __init__(self):
        self.manager = multiprocessing.Manager()
        self._active_nodes = self.manager.list()
        self._lock = self.manager.Lock()

    def __del__(self):
        self.manager.shutdown()

    def start(self, launch_description: LaunchDescription):
        """
        Starts the launch process for the given launch description in a separate process.
        """
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=self._run_process, args=(self._stop_event, launch_description), daemon=True)
        self._process.start()

    def shutdown(self):
        """
        Signals the launch process to shut down and waits for it to terminate.
        """
        self._stop_event.set()
        self._process.join(timeout=20.0)
        if self._process.is_alive():
            self._process.terminate()
            print("The process did not terminate gracefully and was terminated forcefully.")

    def _event_handler(self, action, event, nodes_list, lock):
        """
        Generic event handler for both process start and exit events.
        """
        with lock:
            if action == 'start':
                nodes_list.append({event.process_name: event.pid})
            elif action == 'exit':
                nodes_list[:] = [node for node in nodes_list if node.get(event.process_name) != event.pid]
        
        print(f"Active Nodes after {action}: {nodes_list}")
        if not nodes_list and action == 'exit':
            self.shutdown()

    def _run_process(self, stop_event, launch_description):
        """
        The target method for the process, running the launch service with event handling.
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        launch_description.add_action(
            RegisterEventHandler(OnProcessStart(on_start=lambda event, context: self._event_handler('start', event, self._active_nodes, self._lock)))
        )
        launch_description.add_action(
            RegisterEventHandler(OnProcessExit(on_exit=lambda event, context: self._event_handler('exit', event, self._active_nodes, self._lock)))
        )

        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())

        

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

