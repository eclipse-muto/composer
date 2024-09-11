# Composer - A Lightweight Orchestration Library for ROS2

Composer is a core component of the Eclipse Muto project, designed to automate large-scale logistics operations using autonomous vehicles and advanced robotics. The Composer acts as an orchestrator for ROS2 nodes, providing provisioning, self-healing, and dynamic management of ROS2 workspaces and applications.

## Documentation
You can visit [Muto Docs](https://eclipse-muto.github.io/docs/docs/muto/) to get a broader point of view about what Eclipse Muto is, what it aims to do and the future plans of it.

## Features

- **Orchestration**: Manages the lifecycle of ROS2 nodes and workspaces.
- **Modular Architecture**: Extensible design for adding custom plugins and functionalities.
- **Dynamic Composition**: Automatically provisions ROS2 nodes based on the stack data retrieved from the twin server.
- **Provisioning**: Manages the dependencies of the ROS2 workspaces with the help of `rosdep` and lets the user add custom installation

## Getting Started
### Prerequisites
- ROS2 Humble (Tested on ROS2 Foxy and newer)
- Docker (for containerized operations)
- Python 3.7+

In ideal conditions, `composer` should not work alone. It is a part of a greater system which is `Eclipse Muto`. You could visit below links (which are just ROS2 packages essentially like composer) to get more information about Eclipse Muto:
-  [agent](https://github.com/eclipse-muto/agent)
-  [core](https://github.com/eclipse-muto/core)
-  [messages](https://github.com/eclipse-muto/messages)

After setting up the above and [composer](#installation), 

To set up `composer` alone, refer to the [installation](#installation) section


### Installation

#### 1- Create a standard ROS2 Workspace:
```bash
mkdir $HOME/eclipse-muto/src -p
export MUTO_WS=$HOME/eclipse-muto/src
```
This document will assume that you have other parts of Eclipse Muto cloned under this workspace. (refer to [getting started](#getting-started) section) 

#### 2- Clone composer:
```bash
cd $MUTO_WS/src
git clone https://github.com/eclipse-muto/composer.git
```

#### 3- Install the dependencies:
```bash
cd $MUTO_WS
rosdep install --from-path src --ignore-src -r -y
```

#### 4- Build the workspace
```bash
cd $MUTO_WS
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch

You could refer to the [launch](docs/launch.md) documentation for this