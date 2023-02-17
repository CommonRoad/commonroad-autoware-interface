# Our DFG-Car: **EDGAR**

## Description
This project builds an interface between commonroad and autoware.universe/core. 

## Table of Contents

- [Introduction of the files](#introduction-of-files)
- [Environment setup](#environment-setup)
- [Docker setup and updates](#docker)
- [Modification to Autoware.Universe](#modifications-to-autowareuniverse)
- [**How to use**](#how-to-use)
- [Code structure](#code-structure)

## Introduction of files
_**cr2autoware:**_
* `cr2autoware.py`: a ROS2 node that subscribes information from autoware and processes information and publishes data to autoware.
* `tf2_geometry_msgs.py`: defines help-functions for transformation between map frame and other frames.
* `utils.py`: used for visualization of planning.

_**config:**_
* `avp.rviz`: configuration for rviz and it defines the name of topic for goal pose: goal_pose_cr.

_**data:**_
* `sample-map-planning/lanelet2_map.osm`: map used by the simulation
* `sample-map-planning/mapconfig.yaml`: config for the map used by the simulation
* `sample-map-planning/pointcloud_map.pcd`: pointcloud data of the map used by the simulation

_**launch:**_
* `test.launch.py`: launchfile for this node 

_**param:**_
* `cr2autoware_param_file.param.yaml`: includes parameters for initialization of map, configuration of vehicle and planner.
* `default_yaml`: includes default parameters for reactive planner.

## Environment setup
* Follow the overall softare installation and launch procedure of TUM-Launch: https://wiki.tum.de/pages/viewpage.action?spaceKey=edgar&title=Overall+Software+Installation+and+Launch
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
| commonroad-vehicle-models | 2.0.0 |
| commonroad-route-planner | 2022.3
| autoware | integrate_cr2autoware_interface:latest |
| tum.launch | 50-integrate-cr2autoware-interface:latest |
| autoware.universe | 50-integrate-cr2autoware-interface:latest |

## Docker 
Here the docker setup is described:

### Creating SSH key
For the first usage, you have to generate the ssh key. Go to the `User Settings` -> 'SSH keys'. First generate the ssh key in your local machine and then add it to your gitlab account. See the intruction [here](https://gitlab.lrz.de/help/user/ssh.md).

### cr2autoware setup
First log in to the **docker registry**: 
* If you don't have an existing SSH key connection, you need to login using your LRZ username and password: `docker login -u "your_lrz_username" -p "your_lrz_pwd" gitlab.lrz.de:5005`
* If you have an existing SSH connection, log in using: `docker login gitlab.lrz.de:5005`

Then to download the dockerimage just run the command to start the container (We have two repositories for the project, run the command for the repository in which you are working):

* _**Option 1:**_ 
`rocker --nvidia --x11 --volume $HOME/workspace/workspace:/root/workspace -- gitlab.lrz.de:5005/cps/dfg-car:latest` 

* _**Option 2:**_ 
`rocker --nvidia --x11 --volume $HOME/workspace/workspace:/root/workspace -- gitlab.lrz.de:5005/av2.0/commonroad/commonroad-autoware-interface:latest`

It will fetch the image from the container registry of this repository.

### cr2autoware update
To update the docker image in the container registry run the following commands in the main repository folder (change the gitlab address if you are working with AV2.0 repository):
1. Do the changes you want to do.
2. `docker login gitlab.lrz.de:5005`
3. `docker build -t gitlab.lrz.de:5005/cps/dfg-car .`
4. `docker push gitlab.lrz.de:5005/cps/dfg-car`

### autoware.universe setup
**First log in to the docker registry** `docker login gitlab.lrz.de:5005`.
Then to download the dockerimage just run the command to start the container (We have two repositories for the project, run the command for the repository in which you are working):

* _**Option 1:**_ 
`rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/cps/dfg-car:autoware-universe` 

* _**Option 2:**_ 
`rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/av2.0/commonroad/commonroad-autoware-interface:autoware-universe`

**Setup the autoware repository in the container** (only if first time setup):
1. Follow the instructions in [set up a workspace](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/): 
    * first run `cd autoware`
    * then go to step 3 of the instructions
2. Follow the instructions in [set up the simulator](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/scenario-simulation/planning-simulation/installation/)

### autoware.universe update

To update the docker image in the container registry run the following commands (change the gitlab address if you are working with AV2.0 repository):
1. **Optional**: pull latest changes from autoware
2. Run `autoware/docker/build.sh`
3. Rename image `docker tag ghcr.io/autowarefoundation/autoware-universe:galactic-latest-cuda gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`
>if the renaming failed, you need to check the new image name using `docker images` and change the `galactic-latest-cuda` to a similar one
5. Upload image `docker push gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`

## Modifications to autoware.universe
### Disable trajectory planning of autoware
1. Within your local workspace:
    * Comment out the planning_simulation part in the launch file: `~/workspace/autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml`
    * Open file '~/workspace/autoware/src/launcher/tum_launch/tum_launch/rviz/tum.rviz' and search for 'Name: Planning', then set 'Enabled: false'

2. Within the autoware container:   
    * Run `cd ~/autoware`
    * Run `colcon build --packages-select autoware_launch && source ~/autoware/install/setup.bash` .

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

On this page is shown how to use the simple simulator to init state of the car and set a goal: https://autowarefoundation.github.io/autoware-documentation/latest/tutorials/ad-hoc-simulation/planning-simulation/

## Code structure
In your local machine, the structure should be like:
```
├── ~/workspace/workspace
│   ├── commonroad-scenario-designer
│   ├── commonroad-search
│   ├── reactive-planner
│   ├── pycharm
│   └── dfg-car
│      ├── src
│      ├── Dockerfile
│      └── docker-entrypoint.sh            
```
