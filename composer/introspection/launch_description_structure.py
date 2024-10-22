import launch
import launch_ros.actions
import os
import asyncio


class LaunchDescriptionStructure:
    """
    Parses a launch file and provides a structured representation of its entities.
    """

    def __init__(self, launch_file_path, launch_file_arguments=None):
        self.launch_file_path = launch_file_path
        self.launch_file_arguments = launch_file_arguments or []
        self.launch_description = None

    def launch_a_launch_file(self):
        """Prepare the launch description for introspection."""
        self.launch_description = launch.LaunchDescription(
            [
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.AnyLaunchDescriptionSource(
                        self.launch_file_path
                    ),
                ),
            ]
        )

    def get_launch_description_data(self):
        """Builds and returns a structured representation of the launch description."""
        if self.launch_description is None:
            raise RuntimeError("Launch description is not initialized.")

        context = launch.LaunchContext()
        # Set up a dummy asyncio loop to prevent AttributeError
        context._set_asyncio_loop(asyncio.get_event_loop())

        # Provide required launch arguments if any (you can customize this method)
        self._provide_required_launch_arguments(
            context, self.launch_description.entities
        )

        data = self._get_entities_data(self.launch_description.entities, context)
        return data

    def _provide_required_launch_arguments(self, context, entities):
        """Collect and set required launch arguments in the context."""
        for entity in entities:
            if isinstance(entity, launch.actions.DeclareLaunchArgument):
                # Check if the argument has a default value
                if entity.default_value is None:
                    default_value = "default_value_for_" + entity.name
                    context.launch_configurations[entity.name] = default_value
                else:
                    context.launch_configurations[entity.name] = (
                        entity.default_value.perform(context)
                    )
            elif isinstance(entity, launch.actions.IncludeLaunchDescription):
                # Visit the included launch description to collect arguments
                entity.visit(context)
                sub_entities = entity.describe_sub_entities()
                if sub_entities:
                    self._provide_required_launch_arguments(context, sub_entities)
            elif isinstance(entity, launch.actions.GroupAction):
                if hasattr(entity, "scoped") and entity.scoped:
                    entity.visit(context)
                self._provide_required_launch_arguments(context, entity.actions)

    def _get_entities_data(self, entities, context):
        """Recursively collect entities data."""
        entities_data = []
        for entity in entities:
            # Visit entities that modify the context without executing processes
            if isinstance(
                entity,
                (
                    launch.actions.DeclareLaunchArgument,
                    launch.actions.SetLaunchConfiguration,
                    launch.actions.IncludeLaunchDescription,
                    launch.actions.GroupAction,
                ),
            ):
                entity.visit(context)
            entity_data = self._get_entity_data(entity, context)
            entities_data.append(entity_data)
        return entities_data

    def _get_entity_data(self, entity, context):
        """Collect data from an individual entity."""
        if isinstance(entity, launch.actions.DeclareLaunchArgument):
            data = {
                "type": "DeclareLaunchArgument",
                "name": entity.name,
                "default_value": (
                    self._resolve_substitution(entity.default_value, context)
                    if entity.default_value
                    else None
                ),
                "description": entity.description,
            }
        elif isinstance(entity, launch.actions.IncludeLaunchDescription):
            source = entity.launch_description_source
            location = self._resolve_substitution(source.location, context)
            data = {
                "type": "IncludeLaunchDescription",
                "location": location,
                "launch_arguments": {},
                "included_entities": [],
            }
            # if launch arguments, collect
            if entity.launch_arguments:
                for key, value in entity.launch_arguments:
                    resolved_key = self._resolve_substitution(key, context)
                    resolved_value = self._resolve_substitution(value, context)
                    data["launch_arguments"][resolved_key] = resolved_value
            entity.visit(context)
            sub_entities = entity.describe_sub_entities()
            if sub_entities:
                data["included_entities"] = self._get_entities_data(
                    sub_entities, context
                )
        elif isinstance(entity, launch_ros.actions.Node):
            package = self._resolve_substitution(
                getattr(entity, "_Node__package", None), context
            )
            executable = self._resolve_substitution(
                getattr(entity, "_Node__node_executable", None), context
            )
            node_name = self._resolve_substitution(
                getattr(entity, "_Node__node_name", None), context
            )
            node_namespace = self._resolve_substitution(
                getattr(entity, "_Node__node_namespace", None), context
            )
            parameters = self._resolve_parameters(
                getattr(entity, "_Node__parameters", None), context
            )
            remappings = getattr(entity, "_Node__remappings", []) or []
            arguments = getattr(entity, "_Node__arguments", []) or []
            data = {
                "type": "Node",
                "package": package,
                "executable": executable,
                "name": node_name,
                "namespace": node_namespace,
                "parameters": parameters,
                "remappings": [
                    (
                        self._resolve_substitution(src, context),
                        self._resolve_substitution(dst, context),
                    )
                    for src, dst in remappings
                ],
                "arguments": [
                    self._resolve_substitution(arg, context) for arg in arguments
                ],
            }
        elif isinstance(entity, launch.actions.GroupAction):
            data = {
                "type": "GroupAction",
                "launch_configurations": {},
                "sub_entities": [],
            }
            if hasattr(entity, "launch_configurations"):
                for key, value in entity.launch_configurations.items():
                    resolved_value = self._resolve_substitution(value, context)
                    data["launch_configurations"][key] = resolved_value
            sub_entities = entity.actions if hasattr(entity, "actions") else []
            data["sub_entities"] = self._get_entities_data(sub_entities, context)
        else:
            # For other entities, collect all public attributes
            data = {
                "type": entity.__class__.__name__,
                "attributes": {},
                "sub_entities": [],
            }
            attrs = [
                attr
                for attr in dir(entity)
                if not callable(getattr(entity, attr)) and not attr.startswith("_")
            ]
            for attr in attrs:
                value = getattr(entity, attr)
                if self._is_substitution(value):
                    value = self._resolve_substitution(value, context)
                data["attributes"][attr] = value
            sub_entities = []
            if hasattr(entity, "describe_sub_entities"):
                sub_entities = entity.describe_sub_entities()
            elif hasattr(entity, "entities"):
                sub_entities = entity.entities
            data["sub_entities"] = self._get_entities_data(sub_entities, context)
        return data

    def _is_substitution(self, value):
        """Determine if a value is a substitution or contains substitutions."""
        if isinstance(value, launch.Substitution):
            return True
        elif isinstance(value, (list, tuple)):
            return any(self._is_substitution(item) for item in value)
        else:
            return False

    def _resolve_substitution(self, substitution, context):
        """Helper method to resolve substitutions."""
        if substitution is None:
            return None
        if isinstance(substitution, launch.Substitution):
            return launch.utilities.perform_substitutions(context, [substitution])
        elif isinstance(substitution, (list, tuple)):
            return "".join(
                [
                    (
                        launch.utilities.perform_substitutions(context, [sub])
                        if isinstance(sub, launch.Substitution)
                        else str(sub)
                    )
                    for sub in substitution
                ]
            )
        else:
            return str(substitution)

    def _resolve_parameters(self, parameters, context):
        """Resolve parameters for a Node."""
        if parameters is None:
            return None
        resolved_params = []
        for param in parameters:
            if isinstance(param, dict):
                resolved_dict = {}
                for key, value in param.items():
                    resolved_key = self._resolve_substitution(key, context)
                    if self._is_substitution(value):
                        resolved_value = self._resolve_substitution(value, context)
                    else:
                        resolved_value = value
                    resolved_dict[resolved_key] = resolved_value
                resolved_params.append(resolved_dict)
            elif self._is_substitution(param):
                resolved_param = self._resolve_substitution(param, context)
                resolved_params.append(resolved_param)
            else:
                resolved_params.append(param)
        return resolved_params
