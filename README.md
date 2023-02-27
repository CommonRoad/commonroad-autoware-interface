# CommonRoad-Autoware Motion Planning Interface

## Description
This project builds an interface between [CommonRoad](https://commonroad.in.tum.de/) and [Autoware.Universe](https://github.com/autowarefoundation/autoware.universe). 

## Table of Contents

- [File structure](#file-structure)
- [Repositories and Dependencies](#repositories-and-dependencies)
- [Setup](#setup) 
- [Modifications to Autoware](#modifications-to-autoware)
- [Push a new docker image](#push-a-new-docker-image)
- [**How to use**](#how-to-use)
- [Functionality](#functionality)

## File structure
```
.
├── cr2autoware
│   ├── cr2autoware.py                        # ROS2 interface node that subscribes/publishes data from/to Autoware
│   ├── velocity_planner.py                   # Generates a high-level (reference) velocity profile for the planner
│   ├── rp_interface.py                       # Contains interface to the reactive planner
│   ├── trajectory_logger.py                  # Stores and loads planned trajectories as CommonRoad solution file
│   ├── tf2_geometry_msgs.py                  # helper functions for transformation between coordinate frames
│   └── utils.py                              # various utility fuctions
├── data                                      # directory to store sample CommonRoad/Lanelet2 maps
├── launch
│   └── cr2autoware_node.launch.xml           # launchfile for CR2Auto ROS node 
├── param
│   ├── cr2autoware_param_file.param.yaml     # config settings for the vehicle, reactive planner, and velocity planner
│   └── default_yaml                          # default config parameters for the reactive planner
├── scripts
│   ├── cr2lanelet.py                         # script for offline conversion of a CR map to Lanelet2 format
│   ├── lanelet2cr.py                         # script for offline conversion of a Lanelet2 map to CR format
├── test                                      # directory containing test scripts and test outputs
```


## Repositories and Dependencies
The current version requires the following repositories as dependencies. Further dependencies are included as pip packages (see `requirements.txt`).

The commonroad dependencies and the CR2Autoware interface are currently included as submodules in `autoware.universe/planning/tum_commonroad_planning/`.

| Tools | Versions|
|-|-|
| cps/commonroad-scenario-designer | 40-load-commonroad-scenarios:latest |
| cps/commonroad-search | master:latest |
| cps/reactive-planner | development:latest |
| AV2.0/autoware | integrate_cr2autoware_interface:latest |
| AV2.0/autoware.universe | 50-integrate-cr2autoware-interface:latest |
| AV2.0/tum_launch | 50-integrate-cr2autoware-interface:latest |


## Setup
The setup is aligned with the Rocker Workflow for the 
[overall software installation and launch procedure](https://wiki.tum.de/display/edgar/Rocker+Workflow) with the following additions.
1. Clone AV2.0 autoware repository and checkout branch *[integrate_cr2autoware_interface](https://gitlab.lrz.de/av2.0/autoware/-/tree/integrate_cr2autoware_interface)*.
2. Follow steps 2-4 of the Rocker Workflow
3. Verify that the pulled autoware.universe and tum_launch repos are on the correct branches (see above)
4. Run setup script for CR2Autoware (replace `<autoware_root>` accordingly)
```shell
. <autoware_root>/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/cr2autoware_install.sh
```
5. Follow steps 5-7 of the Rocker Workflow


## Usage and Launch
The interface can be launched within the AW planning simulator via one launch command. We differentiate between two use-cases:

1. **Motion planning mode**: Here, the reactive planner can be used for motion planning on any map or CommonRoad scenario
stored in `<map_directory>`. If the folder contains a CommonRoad scenario, additional scenario information (e.g., initial state,
   goal pose, obstacles...) are loaded from the CR scenario.
```shell
ros2 launch tum_launch planning_simulator.launch.xml map_path:=<map_directory> \
vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
2. **Trajectory replay mode**: Here, a previously planned and stored trajectory in the CommonRoad format can be loaded 
   and followed by the controller. The solution file is stored in `<map_directory>/solutions/solution.xml`
```shell
ros2 launch tum_launch cr2autoware.launch.xml map_path:=<map_directory> \
vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit \
solution_file:="sample-map-planning/solutions/solution1.xml"
```


## Development

### Push a new docker image
To update the docker image in the GitLab container registry, run the following commands (change the GitLab address if you are working with a different repository, e.g., the AV2.0 repo):

1. Make the desired changes to your code
2. Build the docker image, e.g., via: `docker build -t autoware_image . -f autoware/docker/tum_docker/Dockerfile`
3. Rename/Tag the image : `docker tag autoware_image gitlab.lrz.de:5005/cps/dfg-car:latest`
4. Push the image to the container registry: `docker push gitlab.lrz.de:5005/cps/dfg-car:latest`

### Modifications to Autoware
See the [TUM-Launch Wiki page](https://gitlab.lrz.de/cps/dfg-car/-/wikis/TUM-Launch) for the list of changes made to 
autoware, autoware.universe and tum.launch for the integration of the interface and how to replicate them.

*Note: When updating the autoware version, make sure that the documented changes aren't overwritten.*

### Functionality
- This page shows how to initialize the state of the ego vehicle and how to set a goal in rviz: https://autowarefoundation.github.io/autoware-documentation/latest/tutorials/ad-hoc-simulation/planning-simulation/
- Store a solution trajectory:
    - Set config parameter `store_trajectory` to True and define a solution file path with the config parameter `store_trajectory_file`
    - The trajectory is stored as soon as the goal is reached
- Replay a solution trajectory: Trajectories can be replayed by adding the path to the solution file as a launch argument, e.g.: `ros2 launch tum_launch cr2autoware.launch.xml map_path:="sample-map-planning" solution_file:="sample-map-planning/solutions/solution1.xml"`. Use the engage button to start/pause/restart the trajectory replaying.
- Some test maps (CommonRoad + OSM (Autoware) + configuration) can be found in `autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/data/test_maps/lanelet2`. If you want to create your own Autoware-compatible maps: [Load CommonRoad scenarios: usage and explanation](https://gitlab.lrz.de/cps/dfg-car/-/wikis/Load-CommonRoad-scenarios:-usage-and-explanation)


## Authors
*In alphabethic order by first name:*

Maintainers: Gerald Würsching, Yuanfei Lin

Contributors: Andrii Chumak, Florian Weiser, Gerald Würsching, Jan Franck, Koray Koca, Yuanfei Lin, Yashuai Yan