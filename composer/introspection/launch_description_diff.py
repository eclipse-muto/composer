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

        # Compare lengths
        if len(entities_a) != len(entities_b):
            differences.append(
                f"Different number of entities at {path}: {len(entities_a)} vs {len(entities_b)}"
            )

        # Compare each entity
        for i, (entity_a, entity_b) in enumerate(zip(entities_a, entities_b)):
            current_path = f"{path}/{entity_a.get('type', 'Unknown')}[{i}]"
            type_a = entity_a.get("type")
            type_b = entity_b.get("type")

            if type_a != type_b:
                differences.append(
                    f"Type mismatch at {current_path}: {type_a} vs {type_b}"
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
            else:
                diff = self._compare_generic_entities(entity_a, entity_b, current_path)
                differences.extend(diff)

        return differences

    def _compare_launch_arguments(self, arg_a, arg_b, path):
        differences = []
        if arg_a["name"] != arg_b["name"]:
            differences.append(
                f"Name mismatch at {path}: {arg_a['name']} vs {arg_b['name']}"
            )
        if arg_a["default_value"] != arg_b["default_value"]:
            differences.append(
                f"Default value mismatch at {path}: {arg_a['default_value']} vs {arg_b['default_value']}"
            )
        if arg_a["description"] != arg_b["description"]:
            differences.append(
                f"Description mismatch at {path}: {arg_a['description']} vs {arg_b['description']}"
            )
        return differences

    def _compare_include_launch_descriptions(self, include_a, include_b, path):
        differences = []
        if include_a["location"] != include_b["location"]:
            differences.append(
                f"Location mismatch at {path}: {include_a['location']} vs {include_b['location']}"
            )
        if include_a["launch_arguments"] != include_b["launch_arguments"]:
            differences.append(
                f"Launch arguments mismatch at {path}: {include_a['launch_arguments']} vs {include_b['launch_arguments']}"
            )
        diff = self._compare_entities(
            include_a["included_entities"],
            include_b["included_entities"],
            path + "/IncludedEntities",
        )
        differences.extend(diff)
        return differences

    def _compare_nodes(self, node_a, node_b, path):
        differences = []
        for key in [
            "package",
            "executable",
            "name",
            "namespace",
            "parameters",
            "remappings",
            "arguments",
        ]:
            if node_a.get(key) != node_b.get(key):
                differences.append(
                    f"{key.capitalize()} mismatch at {path}: {node_a.get(key)} vs {node_b.get(key)}"
                )
        return differences

    def _compare_group_actions(self, group_a, group_b, path):
        differences = []
        if group_a["launch_configurations"] != group_b["launch_configurations"]:
            differences.append(
                f"Launch configurations mismatch at {path}: {group_a['launch_configurations']} vs {group_b['launch_configurations']}"
            )
        diff = self._compare_entities(
            group_a["sub_entities"], group_b["sub_entities"], path + "/SubEntities"
        )
        differences.extend(diff)
        return differences

    def _compare_generic_entities(self, entity_a, entity_b, path):
        differences = []
        if entity_a.get("attributes") != entity_b.get("attributes"):
            differences.append(
                f"Attributes mismatch at {path}: {entity_a.get('attributes')} vs {entity_b.get('attributes')}"
            )
        diff = self._compare_entities(
            entity_a.get("sub_entities", []),
            entity_b.get("sub_entities", []),
            path + "/SubEntities",
        )
        differences.extend(diff)
        return differences
