import subprocess
from rclpy.node import Node

class Introspector(Node):
    """
    A ROS 2 node for introspecting and managing processes.
    """
    def __init__(self):
        """Initialize the introspector node."""
        super().__init__('introspector')

    def kill(self, name, pid):
        """
        Kills a process by its PID.

        Args:
            name: The name of the process to kill.
            pid: The process ID of the process to kill.
        """
        self.get_logger().info(f'Attempting to kill {name} with PID: {pid}')
        try:
            result = subprocess.run(['kill', str(pid)], check=True, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f'Successfully killed {name} with PID: {pid}')
            else:
                self.get_logger().error(f'Failed to kill {name} with PID: {pid}. Return code: {result.returncode}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Kill was not successful for {name}. Error: {e.stderr}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error while trying to kill {name}. Exception message: {e}')