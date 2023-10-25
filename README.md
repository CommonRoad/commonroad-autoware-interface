# CommonRoad-Autoware Motion Planning Interface

## Description

This project builds an interface between [CommonRoad](https://commonroad.in.tum.de/) and [Autoware.Universe](https://github.com/autowarefoundation/autoware.universe).

## Table of Contents

- [CommonRoad-Autoware Motion Planning Interface](#commonroad-autoware-motion-planning-interface)
  - [Description](#description)
  - [Table of Contents](#table-of-contents)
  - [File structure](#file-structure)
  - [Repositories and Dependencies](#repositories-and-dependencies)
  - [Setup](#setup)
  - [Usage and Launch](#usage-and-launch)
    - [Motion planning mode](#motion-planning-mode)
    - [Trajectory replay mode](#trajectory-replay-mode)
    - [Functionality](#functionality)
  - [Authors](#authors)

## File structure

```tree
.
├── cr2autoware
│   ├── cr2autoware.py                        # ROS2 interface node that subscribes/publishes data from/to Autoware
│   ├── velocity_planner.py                   # Generates a high-level (reference) velocity profile for the planner
│   ├── trajectory_planner_interface.py       # Abstract Base class (ABC) for trajectory planner interface
│   ├── rp_interface.py                       # Interface to the reactive planner (inherited from ABC)
│   ├── scenario_handler.py                   # Handler class to update CR scenario from Autoware scenario during runtime 
│   ├── ego_vehicle_handler.py                # Handler class to update ego vehicle state during runtime 
│   ├── planning_problem_handler.py           # Handler class to update planning problem (initial state + goal) during runtime
│   ├── trajectory_logger.py                  # Stores and loads planned trajectories as CommonRoad solution file
│   ├── tf2_geometry_msgs.py                  # helper functions for transformation between coordinate frames
│   └── utils.py                              # various utility fuctions
├── data                                      # directory to store sample CommonRoad/Lanelet2 maps
├── launch
│   └── cr2autoware_node.launch.xml           # launchfile for CR2Auto ROS node
├── param
│   ├── cr2autoware_param_file.param.yaml     # ROS config settings for the vehicle, reactive planner, and velocity planner
│   └── default_yaml                          # default config parameters for the reactive planner
├── scripts
│   ├── cr2lanelet.py                         # script for offline conversion of a CR map to Lanelet2 format
│   ├── lanelet2cr.py                         # script for offline conversion of a Lanelet2 map to CR format
├── test                                      # directory containing test scripts and test outputs
```

_(`resource` folder and `package.xml` are required for being recognized as a node by ROS2)_

## Repositories and Dependencies

The current version requires the following repositories as dependencies. Further dependencies are included as pip packages (see `requirements.txt`).

The commonroad dependencies and the CR2Autoware interface are currently included as submodules in `autoware.universe/planning/tum_commonroad_planning/`.

| Tools                            | Versions                                  |
| -------------------------------- | ----------------------------------------- |
| cps/commonroad-scenario-designer | 40-load-commonroad-scenarios:latest       |
| cps/commonroad-search            | master:latest                             |
| cps/reactive-planner             | 7c863397                                  |
| AV2.0/autoware                   | integrate_cr2autoware_interface:latest    |
| AV2.0/autoware.universe          | 50-integrate-cr2autoware-interface:latest |
| AV2.0/tum_launch                 | 50-integrate-cr2autoware-interface:latest |

## Setup

For an in-depth guide on the setup for developers, refer to `README_FOR_DEVELOPERS.md`

## Usage and Launch

The interface can be launched within the AW planning simulator via one launch command. We differentiate between two main use-cases:

### Motion planning mode

Here, the CommonRoad interface can be used for motion planning on any map or CommonRoad scenario
stored in `<map_directory>`. Currently, the reactive-planner is used per default, however, the interface will be extended to run
with arbitrary planners in CommonRoad.
If the folder contains a CommonRoad scenario, additional scenario information (e.g., initial state, goal pose, obstacles...)
are loaded from the CR scenario.

```shell
ros2 launch tum_launch planning_simulator.launch.xml map_path:=<map_directory> \
vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

### Trajectory replay mode

Here, a previously planned and stored trajectory in the CommonRoad format can be loaded
and followed by the controller. The solution file is stored in `<map_directory>/solutions/solution.xml`

```shell
ros2 launch tum_launch cr2autoware.launch.xml map_path:=<map_directory> \
vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit \
solution_file:="<map_directory>/solutions/solution.xml"
```

### Functionality

- [This page](https://autowarefoundation.github.io/autoware-documentation/latest/tutorials/ad-hoc-simulation/planning-simulation/)
  shows how to manually place an initial ego state and goal pose within the planning simulator in RVIZ
- Storing a solution trajectory in motion planning mode:
  - Set config parameter `store_trajectory=True` set the solution file path via `store_trajectory_file` in the cr2autoware_param_file
  - The driven trajectory of the ego vehicle is automatically stored as soon as the goal is reached
- Replay a solution trajectory: Trajectories can be replayed by adding the path to the solution file as a launch argument, e.g.: `ros2 launch tum_launch cr2autoware.launch.xml map_path:="sample-map-planning" solution_file:="sample-map-planning/solutions/solution1.xml"`. Use the engage button to start/pause/restart the trajectory replaying.
- Some test maps (CommonRoad + OSM (Autoware) + configuration) can be found in `autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/data/test_maps/lanelet2`. If you want to create your own Autoware-compatible maps: [Load CommonRoad scenarios: usage and explanation](https://gitlab.lrz.de/cps/dfg-car/-/wikis/Load-CommonRoad-scenarios-usage-and-explanation)

## Authors

**In alphabethic order by last name:**

Maintainers: Yuanfei Lin, Gerald Würsching

Contributors: Andrii Chumak, Jan Franck, Koray Koca, Yuanfei Lin, Florian Weiser, Gerald Würsching, Yashuai Yan
