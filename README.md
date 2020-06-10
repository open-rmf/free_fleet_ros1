![](https://github.com/osrf/free_fleet/workflows/build/badge.svg)

# Free Fleet

## Contents

- **[About](#About)**
  - [Client and Server](#client-and-server)
  - [Server Panel](#server-panel)
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Build Instructions](#build-instructions)
  - [Message Generation](#message-generation)
- **[Examples](#examples)**
  - [Multi Turtlebot3 Simulation](#multi-turtlebot3-simulation)
- **[FAQ](#faq)**
- **[Plans](#plans)**

</br>

## About

Welcome to `free_fleet`, an open-source robot fleet management system. 
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

**Note**, this repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

### Client and Server

Free fleet contains two main components, the Client and the Server, which currently uses `CycloneDDS` to communicate between each other. The API to create a client or a server can be found in the `free_fleet` package.

The `ff_client` package contains a client node implementation that is wrapped with ROS1, in order to work with robots using ROS1 navigation stacks. Specifically those that use `move_base`. Its parameters can be modified using `rosparam` during start-up.

### Server Panel

The `ff_rviz_plugins` package contains a panel plugin and relay node to be used with `rviz`, as a developer user interface/server. This allows the user to monitor the states of various robots controlled by free fleet, as well as issue navigational or mode commands to individual robots.

The panel plugin is implemented holding an instance of the free fleet server, which allows it to issue commands from the user to the individual robots, while at the same time provide visual information regarding the states of the robots under free fleet. Its parameters can be modified using `rosparam` during start-up.

Note, a relay server node is currently needed to pass the incoming robot states over ROS1 to the panel, as there are some memory allocation issues between `CycloneDDS` and `rviz`. Alternatives are currently being tested.

</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/)
* [ROS1 - Melodic](https://wiki.ros.org/melodic)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget \
  python-rosdep \
  python-catkin-tools \
  python3-vcstool \
  python3-colcon-common-extensions \
  maven default-jdk   # CycloneDDS dependencies
```

### Build Instructions

Start a new ROS1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/ff_ws/src
cd ~/ff_ws/src

# set up a ROS1 workspace
wget https://raw.githubusercontent.com/osrf/free_fleet/melodic-devel/free_fleet.repos
vcs import src < free_fleet.repos
```

Install all the dependencies through `rosdep`,

```bash
cd ~/ff_ws
source /opt/ros/melodic/setup.bash
rosdep install --from-paths src --ignore-src -y -r
```

Source ROS1 and build,

```bash
cd ~/ff_ws
source /opt/ros/melodic/setup.bash
catkin build
```

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/FleetMessages.idl).

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>

## Examples

### Multi Turtlebot3 Simulation

This example launches three Turtlebot3s in simulation, and registers each robot as a client within a fleet controlled by `free_fleet`.

The launch script starts the `gazebo` simulation, `rviz` visualization with a free fleet panel, a relay server node, navigation stacks of 3 Turtlebot3's, and free fleet clients for each of the robots.

```bash
source ~/ff_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples multi_turtlebot3_ff.launch
```

Once the simulation and visualization show up, the robots can then be commanded to navigate to different parts of the map by using the tool and panels in the visualization. The fleet name can be modified using `rosparam` in the launch file, while the robot name can be selected in the drop-down list. 

Select the navigation waypoints using the default `rviz` tool `2D Nav Goal`, which will be reflected on the panel, while the various buttons handle clearing, deleting waypoints and issuing commands to the specified robots.

![](media/multi_tb3.gif)

</br>

## FAQ

Answers to frequently asked questions can be found [here](/docs/faq.md).
