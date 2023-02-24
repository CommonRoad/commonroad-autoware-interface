# CommonRoad-Autoware Motion Planning Interface

## Description
This project builds an interface between [CommonRoad](https://commonroad.in.tum.de/) and [Autoware.Universe](https://github.com/autowarefoundation/autoware.universe). 

## Table of Contents

- [File structure](#file-structure)
- [Setup](#setup)
- [Repositories and Dependencies](#repositories-and-dependencies)  
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
```


## Setup
* Follow the [overall software installation and launch procedure](https://wiki.tum.de/display/edgar/Rocker+Workflow) (Rocker Workflow) of TUM-Launch: 
**Important**: In the installation instructions, you have the option to build the docker image yourself or to pull it from the CI pipeline. It you decide for the latter option, change the url to: `gitlab.lrz.de:5005/cps/dfg-car:latest`! This docker image also includes the cr2autoware dependencies.
* Make sure that the used autoware, autoware.universe and tum.launch repositories are checked out on the correct feature branches (see table below)
* Initialize and update the submodules via `git submodule update --init`

## Repositories and Dependencies
The current version requires the following repositories as dependencies. Further dependencies are included as pip packages (`requirements.txt`).

| Tools | Versions|
|-|-|
| commonroad-scenario-designer | 40-load-commonroad-scenarios:latest |
| commonroad-search | master:latest |
| reactive-planner | development:latest |
| autoware | integrate_cr2autoware_interface:latest |
| autoware.universe | 50-integrate-cr2autoware-interface:latest |
| tum.launch | 50-integrate-cr2autoware-interface:latest |

## Modifications to Autoware
See the TUM-Launch wiki for a list of changes performed on autoware, autoware.universe and tum.launch and how to replicate them: https://gitlab.lrz.de/cps/dfg-car/-/wikis/TUM-Launch.

When updating the autoware version, make sure that the documented changes aren't overwritten.

## Push a new docker image

To update the docker image in the gitlab container registry, run the following commands (change the gitlab address if you are working with the AV2.0 repository):
0. Perform your changes on the code
1. Build the docker image, e.g. with: `docker build -t autoware_image . -f autoware/docker/tum_docker/Dockerfile`
2. Rename image : `docker tag autoware_image gitlab.lrz.de:5005/cps/dfg-car:latest`
3. Upload image `docker push gitlab.lrz.de:5005/cps/dfg-car:latest`

## How to use
### Option 1: Use one terminal
1. Run the docker image using: `rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware -- <dockerimage>`
2. Source autoware: `source install/setup.bash`
3. Launch autoware and the interface together: `ros2 launch tum_launch planning_simulator.launch.xml map_path:=sample-map-planning/ vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit`

### Option 2: Use two terminals
To view the debug output of Autoware better, it is helpful to have separate terminals for Autoware and the Cr2Autoware interface

0. Comment out the cr2autoware part in the launch file (`src/launcher/tum_launch/tum_launch/launch/cr2autoware.launch.xml`)
1. (Terminal 1) Run the docker image using: `rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware -- <dockerimage>`
2. (Terminal 1) Source autoware: `source install/setup.bash`
3. (Terminal 1) Launch autoware: `ros2 launch tum_launch planning_simulator.launch.xml map_path:=sample-map-planning/ vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit`
4. (Terminal 2) Connect to the docker container: `docker exec -it <container_id> bash`
5. (Terminal 2) Source autoware: `source install/setup.bash`
6. (Terminal 2) Launch cr2autoware: `ros2 launch tum_launch cr2autoware.launch.xml map_path:="sample-map-planning"`

## Functionality
- This page shows how to initialize the state of the ego vehicle and how to set a goal in rviz: https://autowarefoundation.github.io/autoware-documentation/latest/tutorials/ad-hoc-simulation/planning-simulation/
- Store a solution trajectory:
    - Set config parameter `store_trajectory` to True and define a solution file path with the config parameter `store_trajectory_file`
    - The trajectory is stored as soon as the goal is reached
- Replay a solution trajectory: Trajectories can be replayed by adding the path to the solution file as a launch argument, e.g.: `ros2 launch tum_launch cr2autoware.launch.xml map_path:="sample-map-planning" solution_file:="sample-map-planning/solutions/solution1.xml"`. Use the engage button to start/pause/restart the trajectory replaying.
- Some test maps (CommonRoad + OSM (Autoware) + configuration) can be found in `autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/data/test_maps/lanelet2`. If you want to create your own Autoware-compatible maps: [Load CommonRoad scenarios: usage and explanation](https://gitlab.lrz.de/cps/dfg-car/-/wikis/Load-CommonRoad-scenarios:-usage-and-explanation)
