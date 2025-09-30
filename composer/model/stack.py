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

import subprocess
import os
import re
import composer.model.node as node
import composer.model.param as param
import composer.model.composable as composable
import rclpy
from composer.introspection.introspector import Introspector
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

NOACTION = 'none'  # possibly PARAMACTION sometime in the future
STARTACTION = 'start'
STOPACTION = 'stop'
LOADACTION = 'load'


class Stack():
    """The class that contains all stack related operations (apply, kill, stack, merge etc.)"""

    def __init__(self, manifest={}, parent=None):
        """Initialize the Stack object.

        Args:
            manifest (dict, optional): The manifest dictionary containing stack details. Defaults to {}.
            parent (object, optional): The parent stack object. Defaults to None.
        """

        self.manifest = manifest
        self.parent = parent
        self.name = manifest.get('name', '')
        self.context = manifest.get('context', '')
        self.stackId = manifest.get('stackId', '')
        self.param = manifest.get('param', [])
        self.arg = self.resolve_args(manifest.get('arg', []))

        params = []
        for pDef in self.param:
            params.append(param.Param(self, pDef))
        self.param = params

        self.initialize()

    def initialize(self):
        """Initialize the stack elements (nodes, composable nodes, parameters etc.)"""

        self.stack = []
        referenced_stacks = self.manifest.get('stack', [])
        # for stackRef in referenced_stacks:
            # stackDef = self.edge_device.stack(stackRef['thingId'])  # TODO: Replace with the twin service call
            # stack = Stack(stackDef, self)
            # self.stack.append(stack)

        self.node = []
        for nDef in self.manifest.get('node', []):
            sn = node.Node(self, nDef)
            self.node.append(sn)

        self.composable = []
        for cDef in self.manifest.get('composable', []):
            sn = composable.Container(self, cDef)
            self.composable.append(sn)

    def compare_nodes(self, other):
        """Compare the nodes of the stack with another stack.

        Args:
            other (Stack): The other stack object to compare with.

        Returns:
            tuple: A tuple containing sets of common, different, and added nodes.
        """
        nodeSet = set(self.flatten_nodes([]))
        otherNodeSet = set(other.flatten_nodes([]))
        common = nodeSet.intersection(otherNodeSet)
        difference = nodeSet.difference(otherNodeSet)
        added = otherNodeSet.difference(nodeSet)
        return common, difference, added

    def compare_composable(self, other):
        """Compare the composable nodes of the stack with another stack.

        Args:
            other (Stack): The other stack object to compare with.

        Returns:
            tuple: A tuple containing sets of common, different, and added composable nodes.
        """
        current_composables = {f"{c.namespace}/{c.name}": c for c in self.flatten_composable([])}
        other_composables = {f"{c.namespace}/{c.name}": c for c in other.flatten_composable([])}
        
        common_keys = current_composables.keys() & other_composables.keys()
        added_keys = other_composables.keys() - current_composables.keys()
        removed_keys = current_composables.keys() - other_composables.keys()
        
        common = [current_composables[key] for key in common_keys]
        added = [other_composables[key] for key in added_keys]
        removed = [current_composables[key] for key in removed_keys]
    
        return common, added, removed
    
    def flatten_nodes(self, list):
        """Flatten the nested structure of nodes in the stack.

        Args:
            list (list): The list to store flattened nodes.

        Returns:
            list: The flattened list of nodes.
        """
        try:
            for n in self.node:
                list.append(n)
            for s in self.stack:
                s.flatten_nodes(list)
            return list
        except Exception as e:
            print(f'Exception occured in flatten_nodes: {e}')

    def flatten_composable(self, list):
        """Flatten the nested structure of composable nodes in the stack.

        Args:
            list (list): The list to store flattened composable nodes.

        Returns:
            list: The flattened list of composable nodes.
        """

        try:
            for c in self.composable:
                list.append(c)
            for s in self.stack:
                s.flatten_composable(list)
            return list
        except Exception as e:
            print(f'Exception occured in flatten_composable: {e}')

    def calculate_ros_params_differences(self, current, other):
        """Calculate differences in ROS parameters between nodes of the current stack and another stack.

        Args:
            current (Stack): The current stack object.
            other (Stack): The other stack object.

        Returns:
            dict: A dictionary containing differences in ROS parameters.
        """
        differences = {}
        for node_i in current.node:
            for node_j in other.node:
                if node_i.exec == node_j.exec and node_i.pkg == node_j.pkg:
                    diff = self.compare_ros_params(
                        node_i.ros_params, node_j.ros_params)
                    if diff:
                        differences[(node_i.name, node_j.name)] = diff
        return differences

    @staticmethod
    def compare_ros_params(params1, params2):
        """Compare ROS parameters between two sets of parameters.

        Args:
            params1 (list): First set of ROS parameters.
            params2 (list): Second set of ROS parameters.

        Returns:
            list: A list containing the differences between the two sets of parameters.
        """
        diff = []

        def params_to_flat_dict(params):
            flat_dict = {}
            for param in params:
                if isinstance(param, dict):
                    for key in param:
                        flat_dict[key] = param.get(key)
            return flat_dict

        dict_params1 = params_to_flat_dict(params1)
        dict_params2 = params_to_flat_dict(params2)

        all_keys = set(dict_params1).union(set(dict_params2))

        for key in all_keys:
            val1 = dict_params1.get(key, None)
            val2 = dict_params2.get(key, None)

            if val1 != val2:
                diff_entry = {'key': key, 'in_node1': val1, 'in_node2': val2}
                diff.append(diff_entry)

        return diff

    def merge(self, other):
        """Merge the current stack with another stack.

        Args:
            other (Stack): The other stack object to merge with.

        Returns:
            Stack: The merged stack object.
        """

        merged = Stack(manifest={}, parent=None)
        self._merge_attributes(merged, other)
        self._merge_nodes(merged, other)
        self._merge_composables(merged, other)
        self._merge_params(merged, other)

        merged.manifest = merged.toManifest()
        return merged

    def _merge_attributes(self, merged, other):
        merged.name = other.name
        merged.context = other.context
        merged.stackId = other.stackId

    def _merge_nodes(self, merged, other):
        common, difference, added = self.compare_nodes(other)

        for node in common:
            node.action = NOACTION
        for node in added:
            node.action = STARTACTION
        for node in difference:
            node.action = STOPACTION
        merged.node = common.union(added).union(difference)

    def _merge_composables(self, merged, other):
        merged.composable = []

        current_containers = {(c.namespace, c.name): c for c in self.composable}
        other_containers = {(c.namespace, c.name): c for c in other.composable}

        # Process added and removed containers
        for key, container in other_containers.items():
            if key not in current_containers:
                # Mark all nodes within new containers as STARTACTION
                for node in container.nodes:
                    node.action = STARTACTION
                merged.composable.append(container)
            else:
                # For existing containers, compare nodes within and mark actions
                current_container = current_containers[key]
                self.compare_and_mark_nodes(current_container, container, merged)

        for key, container in current_containers.items():
            if key not in other_containers:
                # Mark all nodes within removed containers as STOPACTION
                for node in container.nodes:
                    node.action = STOPACTION
                merged.composable.append(container)

        return merged
    
    def compare_and_mark_nodes(self, current_container, other_container, merged):
        current_nodes = {(n.namespace, n.name): n for n in current_container.nodes}
        other_nodes = {(n.namespace, n.name): n for n in other_container.nodes}

        for key, node in other_nodes.items():
            if key not in current_nodes:
                node.action = STARTACTION
            else:
                node.action = NOACTION 

        for key, node in current_nodes.items():
            if key not in other_nodes:
                node.action = STOPACTION
            else:

                if node.action != STARTACTION:
                    node.action = NOACTION

        # Add processed nodes back into their respective containers
        processed_container = other_container if other_container in merged.composable else current_container
        processed_container.nodes = list(current_nodes.values()) + [n for n in other_nodes.values() if n.action == STARTACTION]
        if processed_container not in merged.composable:
            merged.composable.append(processed_container)



    def _merge_params(self, merged, other):
        other_params = {param.name: param.value for param in other.param}
        for pn, pv in other_params.items():
            merged.param.append(param.Param(self, {"name": pn, "value": pv}))
        merged.arg = other.arg

    def get_active_nodes(self):
        """Get a list of active nodes.

        Returns:
            list: A list of active nodes.
        """
        n = rclpy.create_node('get_active_nodes', enable_rosout=False)
        n_list = n.get_node_names_and_namespaces()
        n.destroy_node()
        return n_list

    def kill_all(self, launcher):
        """Kill all active nodes which were launched by Muto.

        Args:
            launcher (object): The launcher object.
        """
        intrspc = Introspector()
        for n in launcher._active_nodes:
            for name, pid in n.items():
                intrspc.kill(name, pid)

    def kill_diff(self, launcher, stack):
        """When apply pipeline runs, kill the difference between two stacks.

        Args:
            launcher (object): The launcher object.
            stack (Stack): The stack object.
        """
        intrspc = Introspector()

        # Kill nodes
        for n in stack.node:
            for e in launcher._active_nodes:
                for exec_name, pid in e.items():
                    if n.exec in exec_name and n.action == STOPACTION:
                        intrspc.kill(exec_name, pid)
                        
        # Kill composables
        for container in stack.composable:
            for cn in container.nodes:
                for e in launcher._active_nodes:
                    for exec_name, pid in e.items():
                        if cn.exec in exec_name and cn.action == STOPACTION:
                            intrspc.kill(exec_name, pid)

    def change_params_at_runtime(self, param_differences):
        """Change parameters at runtime based on differences.
        ### TODO: replace with set param service call
        Args:
            param_differences (dict): Dictionary containing parameter differences.
        """
        try:
            for key, val in param_differences.items():
                for i in range(len(val)):
                    subprocess.run(['ros2', 'param', 'set', str(key[0]), str(
                        val[i]['key']), str(val[i]['in_node1'])])
        except Exception as e:
            print(
                f'Exception occurred while changing parameters at runtime: {e}')

    def toShallowManifest(self):
        manifest = {"name": self.name,
                    "context": self.context,
                    "stackId": self.stackId,
                    "param": [],
                    "arg": [],
                    "stack": [],
                    "composable": [],
                    "node": []}
        return manifest

    def toManifest(self):
        """Convert the stack to a manifest dictionary.

        Returns:
            dict: Manifest dictionary representing the stack.
        """
        manifest = self.toShallowManifest()
        for p in self.param:
            manifest["param"].append(p.toManifest())
        for a in self.arg:
            manifest["arg"].append(self.arg[a])
        for s in self.stack:
            manifest["stack"].append(s.toShallowManifest())
        for n in self.node:
            manifest["node"].append(n.toManifest())
        for c in self.composable:
            manifest["composable"].append(c.toManifest())
        return manifest

    def process_remaps(self, remaps_config):
        """Process remaps configuration.

        Args:
            remaps_config (list): List of remaps configurations.

        Returns:
            list: List of processed remaps.
        """
        return [(rmp['from'], rmp['to']) for rmp in remaps_config] if remaps_config else []

    def should_node_run(self, node, launcher):
        """Check if a node should run. 
        This method clears the situation where a 
        node has NOACTION but it isn't running
        NOACTION is meant to keep the common processes alive when switching stacks

        Args:
            node (object): The node object.
            launcher (object): The launcher object.

        Returns:
            bool: True if the node should run, False otherwise.
        """
        active_nodes = [(active[1] if active[1] != '/' else '') +
                        '/' + active[0] for active in launcher._active_nodes]
        
        should_node_run = f'{node.namespace}/{node.name}' not in active_nodes 
        return should_node_run

    def load_common_composables(self, container, launch_description: LaunchDescription):
        """If there are common containers in stack composables, load them onto the existing container
        Args: 
            container (object): The container object
        """
        node_desc = []
        for cn in container.nodes:
            if cn.action == LOADACTION:
                print(f"LOADING {cn.namespace}/{cn.name}")
                node_desc.append(ComposableNode(
                    package=cn.pkg,
                    name=cn.name,
                    namespace=cn.namespace,
                    plugin=cn.plugin
                ))

        if node_desc:
            load_action = LoadComposableNodes(
                target_container=f'{container.namespace}/{container.name}',
                composable_node_descriptions=[node_desc],
            )
            launch_description.add_action(load_action)

    def handle_composable_nodes(self, composable_containers, launch_description, launcher):
        """Handle composable nodes during stack launching.

        Args:
            composable_containers (list): List of composable nodes.
            launch_description (object): The launch description object.
        """
        for c in composable_containers:
            node_desc = [ComposableNode(package=cn.pkg, plugin=cn.plugin, name=cn.name, namespace=cn.namespace, parameters=cn.ros_params, remappings=self.process_remaps(cn.remap))
                         for cn in c.nodes if cn.action == STARTACTION or (cn.action == NOACTION and self.should_node_run(cn, launcher))]

            if node_desc:  # If node_desc is not empty
                container = ComposableNodeContainer(
                    name=c.name,
                    namespace=c.namespace,
                    package=c.package,
                    executable=c.executable,
                    output=c.output,
                    composable_node_descriptions=node_desc,
                )
                launch_description.add_action(container)

            # self.load_common_composables(c, launch_description)

    def handle_regular_nodes(self, nodes, launch_description, launcher):
        """Handle regular nodes during stack launching.

        Args:
            nodes (list): List of regular nodes.
            launch_description (object): The launch description object.
        """
        for n in nodes:
            action = n.action
            if action == "":
                action = NOACTION
            if action == STARTACTION or (action == NOACTION and self.should_node_run(n, launcher)):
                launch_description.add_action(Node(
                    package=n.pkg,
                    executable=n.exec,
                    name=n.name,
                    namespace=n.namespace,
                    output=n.output,
                    parameters=n.ros_params,
                    arguments=n.args.split(),
                    remappings=self.process_remaps(n.remap)
                ))

    def handle_managed_nodes(self, nodes, verb):
        """Handle regular nodes during stack launching.

        Args:
            nodes (list): List of lifecycle nodes.
            launch_description (object): The launch description object.
        """
        for n in nodes:
            if n.lifecycle:
                verbs = n.lifecycle.get(verb, [])
                n.change_state(verbs=verbs)

    def launch(self, launcher):
        """Launch the stack.

        Args:
            launcher (object): The launcher object.
        """
        launch_description = LaunchDescription()

        try:
            self.handle_composable_nodes(
                self.composable, launch_description, launcher)
            self.handle_regular_nodes(self.node, launch_description, launcher)

        except Exception as e:
            print(f'Stack launching ended with exception: {e}')

        launcher.start(launch_description)
        all_nodes = self.node + [cn for c in self.composable for cn in c.nodes]

        # After nodes are launched, take care of managed node actions
        self.handle_managed_nodes(all_nodes, verb='start')  

    def apply(self, launcher):
        """Apply the stack.

        Args:
            launcher (object): The launcher object.
        """
        self.kill_diff(launcher, self)
        self.launch(launcher)

    def resolve_expression(self, value=""):
        """Resolve Muto expressions like find, arg, etc.

        Args:
            value (str, optional): The value containing expressions. Defaults to "".

        Returns:
            str: The resolved value.
        """
        value = str(value)
        expressions = re.findall(r'\$\(([\s0-9a-zA-Z_-]+)\)', value)
        result = value

        for expression in expressions:
            expr, var = expression.split()
            resolved_value = ""

            try:
                if expr == 'find':
                    resolved_value = get_package_share_directory(var)
                elif expr == 'env':
                    resolved_value = os.environ[var]
                elif expr == 'optenv':
                    resolved_value = os.environ.get(var, '')
                elif expr == 'arg':
                    arg_value = self.arg.get(var)
                    if arg_value is not None:
                        resolved_value = arg_value['value']
                elif expr == 'eval':
                    raise NotImplementedError(
                        f"Value: {value} is not supported in Muto")
                else:
                    continue

                result = re.sub(
                    r'\$\(' + re.escape(expression) + r'\)', resolved_value, result, count=1)
            except KeyError:
                raise Exception(f"{var} does not exist", 'param')
            except Exception as e:
                print(f'Exception occurred: {e}')

        return result

    def resolve_param_expression(self, param={}):
        name = ''
        value = None
        valKey = None
        for k in param.keys():
            if 'name' == k:
                name = param['name']
            else:
                value = param[k]
                valKey = k
        if not valKey is None:
            return (name, valKey, self.resolve_expression(value))
        return None

    def resolve_args(self, array=[]):
        result = {}
        self.arg = {}
        for item in array:
            name, key, value = self.resolve_param_expression(item)
            p = {"name": name}
            p[key] = value
            result[name] = p
            self.arg[name] = p

        return result