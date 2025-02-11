import os
import composer.model.param as param
from lifecycle_msgs.msg import Transition, State
from lifecycle_msgs.srv import GetState, GetAvailableTransitions, GetAvailableStates, ChangeState
import rclpy

class Node:
    def __init__(self, stack, manifest={}, container=None):
        if manifest is None:
            manifest = {}

        self.stack = stack
        self.container = container
        self.manifest = manifest
        self.env = manifest.get('env', [])
        self.param = [param.Param(stack, pDef) for pDef in manifest.get('param', [])]
        self.remap = manifest.get('remap', [])
        self.pkg = manifest.get('pkg', '')
        self.exec = manifest.get('exec', '')
        self.plugin = manifest.get('plugin', '')
        self.lifecycle = manifest.get('lifecycle', '')
        self.name = manifest.get('name', '')
        self.ros_args = manifest.get('ros_args', '')
        self.args = stack.resolve_expression(manifest.get('args', ''))
        self.namespace = manifest.get('namespace', os.getenv('MUTONS', ''))
        self.launch_prefix = manifest.get('launch-prefix', None)
        self.output = manifest.get('output', 'both')
        self.iff = manifest.get('if', '')
        self.unless = manifest.get('unless', '')
        self.action = manifest.get('action', '')
        self.ros_params = [{key: value} for p in self.param if isinstance(p.value, dict) for key, value in p.value.items() ]
        self.remap_args = [(stack.resolve_expression(rm['from']), stack.resolve_expression(rm['to'])) for rm in self.remap]

    def toManifest(self):
        """Converts the node object back into a manifest dictionary."""
        return {
            "env": self.env,
            "param": [p.toManifest() for p in self.param],
            "remap": [{"from": rm[0], "to": rm[1]} for rm in self.remap_args],
            "pkg": self.pkg,
            "lifecycle": self.lifecycle,
            "exec": self.exec,
            "plugin": self.plugin,
            "name": self.name,
            "ros_args": self.ros_args,
            "args": self.args,
            "namespace": self.namespace,
            "launch-prefix": self.launch_prefix,
            "output": self.output,
            "if": self.iff,
            "unless": self.unless,
            "action": self.action
        }
    
    def change_state(self, verbs=[]):
        if self.lifecycle:
            temporary_node = rclpy.create_node('change_state_node')
            state_cli = temporary_node.create_client(ChangeState, f'/{self.namespace}/{self.name}/change_state')
            while not state_cli.wait_for_service(timeout_sec=1.0):
                temporary_node.get_logger().warn('Lifecycle change state service not available. Waiting...')
            
            for verb in verbs:
                request = ChangeState.Request()
                t = Transition()
                t.label = verb
                request.transition = t
                future = state_cli.call_async(request)
                rclpy.spin_until_future_complete(temporary_node, future, timeout_sec=3.0)
            temporary_node.destroy_node()
        else:
            print(f"{self.name} is Not a managed node")


    def get_state(self):
        if self.lifecycle:
            temporary_node = rclpy.create_node('get_state_node')
            state_cli = temporary_node.create_client(GetState, f'/{self.namespace}/{self.name}/get_state')
            while not state_cli.wait_for_service(timeout_sec=1.0):
                temporary_node.get_logger().warn('Lifecycle get state service not available. Waiting...')
            request = GetState.Request()
            future = state_cli.call_async(request)
            rclpy.spin_until_future_complete(temporary_node, future, timeout_sec=3.0)
            temporary_node.destroy_node()
            return future.result()
        else:
            print(f"{self.name} is Not a managed node")

    def get_available_states(self):
        if self.lifecycle:
            temporary_node = rclpy.create_node('get_available_states_node')
            state_cli = temporary_node.create_client(GetAvailableStates, f'/{self.namespace}/{self.name}/get_available_states')
            while not state_cli.wait_for_service(timeout_sec=1.0):
                temporary_node.get_logger().warn('Lifecycle get_available_states service not available. Waiting...')
            request = GetAvailableStates.Request()
            response = GetAvailableStates.Response()
            future = state_cli.call_async(request)
            rclpy.spin_until_future_complete(temporary_node, future, timeout_sec=3.0)
            response = future.result()
            temporary_node.destroy_node()
            return response.available_states
        else:
            print(f"{self.name} is Not a managed node")


    def __eq__(self, other):
        """Checks if two Node objects are equal based on their attributes."""
        return isinstance(other, Node) and all(
            getattr(self, attr) == getattr(other, attr) for attr in [
                'pkg', 'name', 'namespace', 'exec', 'plugin', 'args'
            ])

    def __hash__(self):
        """Computes a hash based on certain attributes of the Node."""
        return hash((self.pkg, self.name, self.namespace, self.exec, self.plugin))