# Composer

**Composer** is a ROS 2 package designed to organize and automate the software deployment process to a fleet of vehicles. It streamlines the workflow by managing stack definitions, resolving dependencies, handling buildjobs and orchestrating the execution of various pipelines.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
  - [Configuration](#configuration)
  - [Launching Composer](#launching-composer)
- [Plugins](#plugins)
  - [Adding a Plugin](#adding-a-plugin)
- [Working with Stacks](#working-with-stacks)
- [Introspection Tools](#introspection-tools)
- [Blueprint](#blueprint)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Overview

Composer automates the deployment process of ROS2 systems by handling the orchestration of different services and plugins. It receives deployment actions from [Muto Agent](https://github.com/eclipse-muto/agent), processes stack definitions, resolves expressions from stacks, and executes pipelines to manage the lifecycle of software stacks on vehicles.

This documentation provides an overview about Composer. Even though you could use Composer alone with some little tweaks, it was intended to be used with other parts of [`Eclipse Muto`](https://eclipse-muto.github.io/docs/docs/muto/). They could be found under [`Agent`](https://github.com/eclipse-muto/agent), [`Core`](https://github.com/eclipse-muto/core) and [`Messages`](https://github.com/eclipse-muto/agent). You could refer to [`Eclipse Muto`](https://eclipse-muto.github.io/docs/docs/muto/) documentation for a detailed guide on how to set Muto up for your use-case.

## Features

- **Automated Deployment Pipelines**: Define and execute custom deployment pipelines.
- **Plugin Architecture**: Easily extend functionality with custom plugins.
- **Software Stack Management**: Handle stack definitions, including cloning repositories, building workspaces, managing dependencies and updating/upgrading the stack.
- **ROS 2 Integration**: Leverage ROS 2 services and messaging for communication between components.
- **Introspection Tools**: Visualize and analyze launch descriptions and pipeline executions.

## Architecture

The Composer package consists of the following main components:

- **Workflow**
  - **Router**: Routes incoming actions to the appropriate pipeline.
  - **Pipeline**: Manages the execution of sequences of steps defined in the pipeline configuration.
- **Plugins**
  - **Compose Plugin**: Parses and publishes stack manifests.
  - **Native Plugin**: Handles workspace preparation, including cloning repositories and building.
  - **Launch Plugin**: Manages the launching and killing of stacks.
- **Introspection Tools**: Tools for visualizing and debugging launch descriptions and pipelines.

For a detailed overview, refer to the [Architecture Documentation](docs/architecture.md).

## Installation

### Prerequisites

- **ROS 2 Foxy** or later installed on your system.
- **Python 3.8** or later.
- Ensure that you have `colcon` and `rosdep` installed for building and dependency management.

```bash
# Add ROS2 apt repository
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install colcon and rosdep
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

### Steps

1. **Clone the Repository**

   ```bash
   cd $HOME
   mkdir -p muto/src
   cd muto/src
   git clone https://github.com/eclipse-muto/agent.git
   git clone https://github.com/eclipse-muto/core.git
   git clone https://github.com/eclipse-muto/composer.git
   git clone https://github.com/eclipse-muto/messages.git
   ```

2. **Install Dependencies**

   Use `rosdep` to install dependencies:

   ```bash
   cd $HOME/muto
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Package**

   Use `colcon` to build the package:

   ```bash
   $HOME/muto
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
## Usage

### Configuration
- For a better workflow, you need to create 2 files: `muto.yaml`, and `muto.launch.py`


```bash
cd $HOME/muto
mkdir config/ && cd config/
# Create the below muto.yaml file under this config directory
```
`muto.yaml`:
```diff
/**:
  ros__parameters:
    stack_topic: "stack"
    twin_topic: "twin"
    agent_to_gateway_topic: "agent_to_gateway"
    gateway_to_agent_topic: "gateway_to_agent"
    agent_to_commands_topic: "agent_to_command"
    commands_to_agent_topic: "command_to_agent"
    thing_messages_topic: "thing_messages"
    - ignored_packages: ["package1", "package3"]  # the packages in this list will be ignored in the build phase
    + ignored_packages: ["package2", "package4"]

    twin_url: "http://ditto:ditto@sandbox.composiv.ai"
    host: sandbox.composiv.ai
    port: 1883
    keep_alive: 60
    user: null
    password: null
    anonymous: false
    type: real_car
    attributes: '{"brand": "muto", "model": "composer"}'
    prefix: muto
    namespace: org.eclipse.muto.sandbox
    name: composer-guy

    commands:
      command1:
        name: ros/topic
        service: rostopic_list
        plugin: CommandPlugin

      command2:
        name: ros/topic/info
        service: rostopic_info
        plugin: CommandPlugin

      command3:
        name: ros/topic/echo
        service: rostopic_echo
        plugin: CommandPlugin

      command4:
        name: ros/node
        service: rosnode_list
        plugin: CommandPlugin

      command5:
        name: ros/node/info
        service: rosnode_info
        plugin: CommandPlugin

      command6:
        name: ros/param
        service: rosparam_list
        plugin: CommandPlugin

      command7:
        name: ros/param/get
        service: rosparam_get
        plugin: CommandPlugin

```

```bash
cd $HOME/muto
mkdir launch/ && cd launch/
# Create the below muto.launch.py file under this launch directory
```
`muto.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    muto_namespace_arg = DeclareLaunchArgument(
        "muto_namespace",
        default_value="muto"
    )

    muto_params = "./config/muto.yaml"

    node_agent = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="agent",
        package="agent",
        executable="muto_agent",
        output="screen",
        parameters=[muto_params]
    )

    node_mqtt_gateway = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="gateway",
        package="agent",
        executable="mqtt",
        output="screen",
        parameters=[muto_params]
    )

    node_commands = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="commands_plugin",
        package="agent",
        executable="commands",
        output="screen",
        parameters=[muto_params]
    )

    node_twin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="core_twin",
        package="core",
        executable="twin",
        output="screen",
        parameters=[muto_params]
    )

    node_composer = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="muto_composer",
        output="screen",
        parameters=[muto_params]
    )

    node_compose_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="compose_plugin",
        output="screen",
        parameters=[
            muto_params,
            {"names": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")}
        ]
    )

    node_native_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="native_plugin",
        output="screen",
         parameters=[muto_params]
    )

    node_launch_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="launch_plugin",
        output="screen",
        parameters=[muto_params]
    )


    ld = LaunchDescription()

    ld.add_action(muto_namespace_arg)
    
    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)
    ld.add_action(node_twin)
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_native_plugin)
    ld.add_action(node_launch_plugin)

    return ld

```

- Composer uses a configuration file `pipeline.yaml` located in the `composer/config` directory to define the pipelines and their steps. Ensure that this file is properly configured to suit your deployment needs.

### Launching

To start `Muto` as a whole (including `composer`):

```bash
cd $HOME/muto
source /opt/ros/$ROS_DISTRO/setup.bash && source install/local_setup.bash
ros2 launch launch/muto.launch.py
```

### Sending Deployment Actions

Composer listens for `MutoAction` messages on the specified stack topic (default is `stack`). You can send deployment actions (e.g., start, kill, apply) to Composer using `Agent` or via a ROS2 CLI tool `ros2 topic pub`.

```bash
ros2 topic pub ...
```


## Plugins

Composer's functionality can be extended through plugins. The default plugins included are:

- **Compose Plugin**: Processes incoming stacks and publishes composed stacks.
- **Native Plugin**: Handles cloning repositories, checking out branches, and building workspaces.
- **Launch Plugin**: Manages the starting and stopping of stacks, including running launch files or scripts.

### Adding a Plugin

To add a new plugin:

1. **Create the Plugin File**

   Place your plugin in the `plugins` directory.

2. **Define the Service Interface**

   Ensure your plugin has a corresponding service definition in `muto_msgs/srv`.

3. **Update the Pipeline Configuration**

   Add your plugin to the `pipeline.yaml` configuration file.

For detailed instructions, refer to the [Adding a Plugin Guide](docs/adding-a-plugin.md).

## Working with Stacks

Stacks are central to Composer's deployment process. A stack definition includes information such as:

- Repository URL and branch
- Launch description source
- Scripts to run on start and kill
- Arguments and environment variables

Ensure your stack definitions are correctly formatted and accessible to Composer.

Refer to the [Working with Stacks Guide](docs/working-with-stacks.md) for more information.

## Introspection Tools

Composer provides tools for introspection and debugging:

- **Launch Description Visualizer**: Visualize the structure of your launch descriptions.
- **Pipeline Execution Monitor**: Monitor the execution of pipelines and steps.

Refer to the [Introspection Tools Documentation](docs/launch.md) for usage instructions.

## Blueprint

## Contributing

Contributions are welcome! Makse sure you follow the [coding guidelines](https://github.com/ibrahimsel/composer/wiki/Coding-Guidelines) that were specified in the project wiki as much as you could.

## License

This project is licensed under the [EPL v2.0](LICENSE).

## Acknowledgements

- **ROS 2 Community**: For providing an excellent framework for robotic software development.
- **Contributors**: Thanks to all the contributors who have helped improve Composer.
