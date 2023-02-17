# Our DFG-Car: **EDGAR**

## Description
This project builds an interface between commonroad and autoware.universe/core. 

## Table of Contents

- [Introduction of the files](#introduction-of-files)
- [Setup](#setup)
- [Modifications to Autoware](#modifications-to-autoware)
- [Push a new docker image](#push-a-new-docker-image)
- [**How to use**](#how-to-use)
- [Functionality](#functionality)

## Introduction of files
_**cr2autoware:**_
* `cr2autoware.py`: a ROS2 node that subscribes information from autoware and processes information and publishes data to autoware.
* `velocity_planner.py`: Used to generate a high-level velocity profile for the planner
* `trajectory_logger.py`: Store and load vehicle trajectories as CommonRoad solution file
* `tf2_geometry_msgs.py`: defines help-functions for transformation between map frame and other frames.
* `utils.py`: used for utility functionality.

_**launch:**_
* `cr2autoware_node.launch.xml`: launchfile for this node 

_**param:**_
* `cr2autoware_param_file.param.yaml`: includes interface settings, configuration variables for the vehicle, reactive planner, and velocity planner.
* `default_yaml`: includes default parameters for the reactive planner.

## Setup
* Follow the overall softare installation and launch procedure of TUM-Launch: https://wiki.tum.de/pages/viewpage.action?spaceKey=edgar&title=Overall+Software+Installation+and+Launch  
**Important**: In the installation instructions, you have the option to build the docker image yourself or to pull it from the CI pipeline. It you decide for the latter option, change the url to: `gitlab.lrz.de:5005/cps/dfg-car:latest`! This docker image also includes the cr2autoware dependencies.
* Make sure that the used autoware, autoware.universe and tum.launch repositories are checked out on the cr2autoware feature branch
* Initialize and update the git submodules

### Repositories used
| Tools | Versions|
|-|-|
| commonroad-scenario-designer | 40-load-commonroad-scenarios:latest |
| commonroad-search | feature-cr2aw:latest |
| reactive-planner | feature_cr_io_new:latest |
| commonroad-io | 2022.3 |
| commonroad-drivability-checker | 2022.2.1 |
| commonroad-route-planner | 2022.3
| autoware | integrate_cr2autoware_interface:latest |
| tum.launch | 50-integrate-cr2autoware-interface:latest |
| autoware.universe | 50-integrate-cr2autoware-interface:latest |

## Modifications to autoware
See the TUM-Launch wiki for a list of changes peformed on autoware, autoware.universe and tum.launch and how to replicate them: https://gitlab.lrz.de/cps/dfg-car/-/wikis/TUM-Launch.

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
- Replay a solution trajectory: Trajectories can be replayed by adding the path to the solution file as a launch argument, e.g.: `ros2 launch tum_launch cr2autoware.launch.xml map_path:="sample-map-planning" solution_file:="sample-map-planning/solutions/solution1.xml"`. Use the engage button to start/pause/restart the trajectory replaying.
