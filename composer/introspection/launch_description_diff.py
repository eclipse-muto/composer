import pprint
from composer.introspection.launch_description_structure import (
    LaunchDescriptionStructure,
)


class LaunchDescriptionDiffer:
    """
    Compares two launch descriptions and identifies differences.
    """

    def __init__(self, launch_file_path_a, launch_file_path_b):
        self.launch_file_path_a = launch_file_path_a
        self.launch_file_path_b = launch_file_path_b
        self.launch_data_a = None
        self.launch_data_b = None

    def load_launch_descriptions(self):
        """Loads and parses the two launch descriptions."""
        visualizer_a = LaunchDescriptionStructure(self.launch_file_path_a)
        visualizer_a.launch_a_launch_file()
        self.launch_data_a = visualizer_a.get_launch_description_data()

        visualizer_b = LaunchDescriptionStructure(self.launch_file_path_b)
        visualizer_b.launch_a_launch_file()
        self.launch_data_b = visualizer_b.get_launch_description_data()

    def diff(self):
        """Computes the differences between the two launch descriptions."""
        differences = self._compare_entities(self.launch_data_a, self.launch_data_b)
        return differences

    def _compare_entities(self, entities_a, entities_b, path=""):
        """Recursively compares entities and returns differences."""
        differences = []

        max_length = max(len(entities_a), len(entities_b))
        for i in range(max_length):
            if i >= len(entities_a):
                entity_b = entities_b[i]
                current_path = (
                    f"{path}/[Extra in B][{i}]/{entity_b.get('type', 'Unknown')}"
                )
                differences.append(
                    f"{current_path}: Entity only in second launch description."
                )
                continue
            elif i >= len(entities_b):
                entity_a = entities_a[i]
                current_path = (
                    f"{path}/[Extra in A][{i}]/{entity_a.get('type', 'Unknown')}"
                )
                differences.append(
                    f"{current_path}: Entity only in first launch description."
                )
                continue

            entity_a = entities_a[i]
            entity_b = entities_b[i]
            current_path = f"{path}/{entity_a.get('type', 'Unknown')}[{i}]"
            type_a = entity_a.get("type")
            type_b = entity_b.get("type")

            if type_a != type_b:
                differences.append(
                    f"{current_path}: Type mismatch: '{type_a}' vs '{type_b}'"
                )
                continue  # Skip further comparison of this entity

            if type_a == "DeclareLaunchArgument":
                diff = self._compare_launch_arguments(entity_a, entity_b, current_path)
                differences.extend(diff)
            elif type_a == "IncludeLaunchDescription":
                diff = self._compare_include_launch_descriptions(
                    entity_a, entity_b, current_path
                )
                differences.extend(diff)
            elif type_a == "Node":
                diff = self._compare_nodes(entity_a, entity_b, current_path)
                differences.extend(diff)
            elif type_a == "GroupAction":
                diff = self._compare_group_actions(entity_a, entity_b, current_path)
                differences.extend(diff)
            elif type_a == "SetEnvironmentVariable":
                diff = self._compare_set_environment_variable(
                    entity_a, entity_b, current_path
                )
                differences.extend(diff)
            elif type_a == "ExecuteProcess":
                diff = self._compare_execute_process(entity_a, entity_b, current_path)
                differences.extend(diff)
            else:
                diff = self._compare_generic_entities(entity_a, entity_b, current_path)
                differences.extend(diff)

        return differences

    def _compare_launch_arguments(self, arg_a, arg_b, path):
        differences = []
        if arg_a["name"] != arg_b["name"]:
            differences.append(
                f"{path}: Name mismatch: '{arg_a['name']}' vs '{arg_b['name']}'"
            )
        if arg_a["default_value"] != arg_b["default_value"]:
            differences.append(
                f"{path}: Default value mismatch: '{arg_a['default_value']}' vs '{arg_b['default_value']}'"
            )
        if arg_a["description"] != arg_b["description"]:
            differences.append(
                f"{path}: Description mismatch: '{arg_a['description']}' vs '{arg_b['description']}'"
            )
        return differences

    def _compare_include_launch_descriptions(self, include_a, include_b, path):
        differences = []
        if include_a["location"] != include_b["location"]:
            differences.append(
                f"{path}: Location mismatch:\n  '{include_a['location']}'\n  vs\n  '{include_b['location']}'"
            )
        if include_a["launch_arguments"] != include_b["launch_arguments"]:
            differences.append(
                f"{path}: Launch arguments mismatch:\n  {pprint.pformat(include_a['launch_arguments'])}\n  vs\n  {pprint.pformat(include_b['launch_arguments'])}"
            )
        # Recursively compare included entities
        diff = self._compare_entities(
            include_a["included_entities"],
            include_b["included_entities"],
            path + "/IncludedEntities",
        )
        differences.extend(diff)
        return differences

    def _compare_nodes(self, node_a, node_b, path):
        differences = []
        for key in ["package", "executable", "name", "namespace"]:
            if node_a.get(key) != node_b.get(key):
                differences.append(
                    f"{path}: {key.capitalize()} mismatch: '{node_a.get(key)}' vs '{node_b.get(key)}'"
                )
        if node_a.get("parameters") != node_b.get("parameters"):
            differences.append(
                f"{path}: Parameters mismatch:\n  {pprint.pformat(node_a.get('parameters'))}\n  vs\n  {pprint.pformat(node_b.get('parameters'))}"
            )
        if node_a.get("remappings") != node_b.get("remappings"):
            differences.append(
                f"{path}: Remappings mismatch:\n  {pprint.pformat(node_a.get('remappings'))}\n  vs\n  {pprint.pformat(node_b.get('remappings'))}"
            )
        if node_a.get("arguments") != node_b.get("arguments"):
            differences.append(
                f"{path}: Arguments mismatch:\n  {pprint.pformat(node_a.get('arguments'))}\n  vs\n  {pprint.pformat(node_b.get('arguments'))}"
            )
        return differences

    def _compare_group_actions(self, group_a, group_b, path):
        differences = []
        if group_a["launch_configurations"] != group_b["launch_configurations"]:
            differences.append(
                f"{path}: Launch configurations mismatch:\n  {pprint.pformat(group_a['launch_configurations'])}\n  vs\n  {pprint.pformat(group_b['launch_configurations'])}"
            )
        diff = self._compare_entities(
            group_a["sub_entities"], group_b["sub_entities"], path + "/SubEntities"
        )
        differences.extend(diff)
        return differences

    def _compare_set_environment_variable(self, env_a, env_b, path):
        differences = []
        if env_a["name"] != env_b["name"]:
            differences.append(
                f"{path}: Environment variable name mismatch: '{env_a['name']}' vs '{env_b['name']}'"
            )
        if env_a["value"] != env_b["value"]:
            differences.append(
                f"{path}: Environment variable value mismatch: '{env_a['value']}' vs '{env_b['value']}'"
            )
        return differences

    def _compare_execute_process(self, proc_a, proc_b, path):
        differences = []
        if proc_a.get("cmd") != proc_b.get("cmd"):
            differences.append(
                f"{path}: Command mismatch:\n  {pprint.pformat(proc_a.get('cmd'))}\n  vs\n  {pprint.pformat(proc_b.get('cmd'))}"
            )
        if proc_a.get("name") != proc_b.get("name"):
            differences.append(
                f"{path}: Name mismatch: '{proc_a.get('name')}' vs '{proc_b.get('name')}'"
            )
        if proc_a.get("env") != proc_b.get("env"):
            differences.append(
                f"{path}: Environment mismatch:\n  {pprint.pformat(proc_a.get('env'))}\n  vs\n  {pprint.pformat(proc_b.get('env'))}"
            )
        return differences

    def _compare_generic_entities(self, entity_a, entity_b, path):
        differences = []
        if entity_a.get("attributes") != entity_b.get("attributes"):
            differences.append(
                f"{path}: Attributes mismatch:\n  {pprint.pformat(entity_a.get('attributes'))}\n  vs\n  {pprint.pformat(entity_b.get('attributes'))}"
            )
        diff = self._compare_entities(
            entity_a.get("sub_entities", []),
            entity_b.get("sub_entities", []),
            path + "/SubEntities",
        )
        differences.extend(diff)
        return differences
