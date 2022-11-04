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
Install **Docker**, **NVIDIA Container Toolkit** and **rocker**. See for that: https://autowarefoundation.github.io/autoware-documentation/latest/installation/autoware/docker-installation/

In the following, we use a folder `~/workspace` to collect all repositories and code. You are free to use any other path for that folder.

1. `mkdir ~/workspace && mkdir ~/workspace/workspace`
2. `cd ~/workspace && git clone https://github.com/autowarefoundation/autoware.git`
3. _**Option 1:**_ `cd ~/workspace/workspace && git clone https://gitlab.lrz.de/cps/dfg-car.git`

   _**Option 2:**_ `cd ~/workspace/workspace && git clone https://gitlab.lrz.de/av2.0/commonroad/commonroad-autoware-interface.git`
4. Clone the following necessary CommonRoad repositories into the `/workspace/workspace` directory (see also the directory structure in section [Code structure](#code-structure)):
    - **commonroad-scenario-designer**: 
        - `git clone https://gitlab.lrz.de/cps/commonroad-scenario-designer.git`
        - `git checkout 13365aa714e61278b57ae6046fa9871ecbab527b`
    - **commonroad-search**:
        - `git clone https://gitlab.lrz.de/tum-cps/commonroad-search.git`
    - **reactive-planner**
        - `git clone -b development https://gitlab.lrz.de/cps/reactive-planner.git`
        - `git checkout e9a68dc3891ee6fd1d2500083c5204384ae94448`
5. Do the setup of the docker containers for [cr2autoware](#cr2autoware-setup) and [autoware.universe](#autowareuniverse-setup).

### Repositories used
| Tools | Versions|
|-|-|
| commonroad-io | >= 2021.4, < 2022.2 |
| commonroad-drivability-checker | 2021.4 |
| commonroad-vehicle-models | 2.0.0 |
| commonroad-search | master:latest |
| commonroad-scenario-designer | development:latest |
| reactive-planner | development:e9a68dc3891ee6fd1d2500083c5204384ae94448 |
| autoware.universe | master:latest |
| autoware.core | master:latest |

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
First log in to the docker registry `docker login gitlab.lrz.de:5005`.
Then to download the dockerimage just run the command to start the container (We have two repositories for the project, run the command for the repository in which you are working):

* _**Option 1:**_ 
`rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/cps/dfg-car:autoware-universe` 

* _**Option 2:**_ 
`rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/av2.0/commonroad/commonroad-autoware-interface:autoware-universe`

> If this is failed with `Failed to build detector image`, 
you have to go to run [autoware universe update](#autowareuniverse-update) without pulling the latest changes.

Setup the autoware repository in the container (only if first time setup):
   - `sudo apt update`
   - `cd ~/autoware`
   - `mkdir src`
   - `vcs import src < autoware.repos`
   - `vcs import src < simulator.repos`
   - `source /opt/ros/galactic/setup.bash`
   - `rosdep update`
   - `rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y`
   - `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
   - `source install/setup.bash`

### autoware.universe update
The docker image provided by autoware does not contain the simulator at the moment. Therefore, we modified the Dockerfile and build our own image of autoware with the simple simulator.

To update the docker image in the container registry run the following commands (change the gitlab address if you are working with AV2.0 repository):
1. **Optional**: pull latest changes from autoware
2. Copy `DockerfileAutowareUniverse` to `autoware/docker/autoware-universe/Dockerfile`
3. Run `autoware/docker/build.sh`
4. Rename image `docker tag ghcr.io/autowarefoundation/autoware-universe:galactic-latest-cuda gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`
>if the renaming failed, you need to check the new image name using `docker images` and change the `galactic-latest-cuda` to a similar one
5. Upload image `docker push gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`

## Modifications to autoware.universe
### Disable trajectory planning of autoware

* Comment out the planning_simulation part in the launch file: `~/workspace/autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml`

Within the autoware container:   
* Run `cd ~/autoware`
* Run `colcon build --packages-select autoware_launch && source ~/autoware/install/setup.bash` .

## How to use
0. Create **2** terminals (maybe Terminator is usefull here)

1. Terminal **1**: open **cr2autoware** container

* _**Option 1:**_ 
`rocker --nvidia --x11 --volume $HOME/workspace/workspace:/root/workspace -- gitlab.lrz.de:5005/cps/dfg-car:latest` 

* _**Option 2:**_ 
`rocker --nvidia --x11 --volume $HOME/workspace/workspace:/root/workspace -- gitlab.lrz.de:5005/av2.0/commonroad/commonroad-autoware-interface:latest` 

- To start the CommonRoad Interface: `ros2 launch cr2autoware test.launch.py`

2. Terminal **2**: open **autoware.universe** container 

* _**Option 1:**_ 
``rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`` 

* _**Option 2:**_ 
``rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/av2.0/commonroad/commonroad-autoware-interface:autoware-universe`` 

- To start autoware simulation: `source ~/autoware/install/setup.bash && ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/workspace/dfg-car/src/cr2autoware/data/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit`

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
