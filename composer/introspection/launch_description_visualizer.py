from graphviz import Digraph
import os


class LaunchDescriptionGraphGenerator:
    """
    Generates a diagrammatic visualization of the launch descriptions and their differences.
    """

    def __init__(self, launch_data_a, launch_data_b, differences):
        self.launch_data_a = launch_data_a
        self.launch_data_b = launch_data_b
        self.differences = differences
        self.graph = Digraph(comment="Launch Description Differences")
        self.path_to_node_id = {}

    def generate_graph(self):
        self._build_graph(self.launch_data_a, "A", current_path="")
        self._build_graph(self.launch_data_b, "B", current_path="")

        # self._highlight_differences()  # TODO

    def _build_graph(self, entities, label_prefix, parent_id=None, current_path=""):
        for i, entity in enumerate(entities):
            entity_type = entity.get("type", "Unknown")
            node_id = f"{label_prefix}_{id(entity)}_{i}"
            label = self._get_entity_label(entity)
            self.graph.node(node_id, label=label)

            path = f"{current_path}/{entity_type}[{i}]"
            self.path_to_node_id[(label_prefix, path)] = node_id

            if parent_id:
                self.graph.edge(parent_id, node_id)

            sub_entities = []
            if entity_type == "IncludeLaunchDescription":
                sub_entities = entity.get("included_entities", [])
            elif entity_type == "GroupAction":
                sub_entities = entity.get("sub_entities", [])
            else:
                sub_entities = entity.get("sub_entities", [])

            if sub_entities:
                self._build_graph(
                    sub_entities, label_prefix, parent_id=node_id, current_path=path
                )

    def _get_entity_label(self, entity):
        entity_type = entity.get("type", "Unknown")
        if entity_type == "Node":
            name = entity.get("name") or "Unnamed"
            label = f"Node: {name}"
        elif entity_type == "IncludeLaunchDescription":
            location = entity.get("location", "Unknown")
            label = f"Include: {os.path.basename(location)}"
        elif entity_type == "DeclareLaunchArgument":
            name = entity.get("name", "Unknown")
            label = f"Arg: {name}"
        else:
            label = entity_type
        return label

    def _highlight_differences(self):
        try:
            for diff in self.differences:
                print(diff)
                path = diff["path"]
                difference_type = diff["difference"]

                node_id_a = self._get_node_id_from_path(path, "A")
                node_id_b = self._get_node_id_from_path(path, "B")

                if difference_type == "only_in_a" and node_id_a:
                    self.graph.node(node_id_a, color="red")
                elif difference_type == "only_in_b" and node_id_b:
                    self.graph.node(node_id_b, color="green")
                elif node_id_a and node_id_b:
                    self.graph.node(node_id_a, color="orange")
                    self.graph.node(node_id_b, color="orange")
        except Exception as e:
            pass

    def _get_node_id_from_path(self, path, label_prefix):
        return self.path_to_node_id.get((label_prefix, path))

    def export_graph(self, filename="launch_diff"):
        self.graph.render(filename, view=False, format="pdf")
        print(f"Graph exported as {filename}")
