# Our DFG-Car: **EDGAR**

## Description
This project builds an interface between commonroad and autoware.universe/core. 

## Introduction of files
cr2autoware
* cr2autoware.py: a ros node that subscribe information from autoware and process information and publish data to autoware
* tf2_geometry_msgs.py: define help-functions for transformation between map frame and other frames.
* utils.py: used for visualization of planning

config
* avp.rviz: configuration for rviz and it defines the name of topic for goal pose: goal_pose_cr.

data:
* sample-map-planning/lanelet2_map.osm: map used by the simulation
* sample-map-planning/mapconfig.yaml: config for the map used by the simulation
* sample-map-planning/pointcloud_map.pcd: pointcloud data of the map used by the simulation

launch:
* test.launch.py: launchfile for this node 

param:
* cr2autoware_param_file.param.yaml: parameter for initialization of map and vehicle etc.

## Environment setup
Install **Docker**, **NVIDIA Container Toolkit** and **rocker**. See for that: https://autowarefoundation.github.io/autoware-documentation/latest/installation/autoware/docker-installation/

In the following we use a folder `~/workspace` to collect all repositories and code. You are free to use any other path for that folder.

1. `mkdir ~/workspace && mkdir ~/workspace/workspace`
2. `cd ~/workspace && git clone https://github.com/autowarefoundation/autoware.git`
3. `cd ~/workspace/workspace && git clone https://gitlab.lrz.de/cps/dfg-car.git`
4. Do the setup of the docker containers for [cr2autoware](#cr2autoware-setup) and [autoware.universe](#autowareuniverse-setup).

### Repositories used
| Tools | Versions|
|-|-|
| commonroad-io | ??? |
| commonroad-drivability-checker | ??? |
| commonroad-vehicle-models | ??? |
| commonroad-search | ??? |
| commonroad-scenrio-designer | development:latest |
| reactive-planner | development:e9a68dc3891ee6fd1d2500083c5204384ae94448 |
| autoware.universe | master:latest |
| autoware.code | master:latest |

## Docker 
Here the docker setup is described:

### cr2autoware setup
First log in to the docker registry `docker login gitlab.lrz.de:5005`.
Then to download the dockerimage just run the comand to start the container: `rocker --nvidia --x11 --volume $HOME/workspace/workspace:/root/workspace -- gitlab.lrz.de:5005/cps/dfg-car`. It will fetch the image from the container registry of this repository.

### cr2autoware update
To update the docker image in the container registry run the following commands in the main repository folder:
1. Do the changes you want to do.
2. `docker login gitlab.lrz.de:5005`
3. `docker build -t gitlab.lrz.de:5005/cps/dfg-car .`
4. `docker push gitlab.lrz.de:5005/cps/dfg-car`

### autoware.universe setup
First log in to the docker registry `docker login gitlab.lrz.de:5005`.
Then to the dockerimage just run the comand to start the container: `rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`. It will fetch the image from the container registry of this repository.

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

To update the docker image in the container registry run the following commands in the main repository folder:
1. **Optional**: pull latest changes from autoware
2. Copy `DockerfileAutowareUniverse` to `autoware/docker/autoware-universe/Dockerfile`
3. Run `autoware/docker/build.sh`
4. Rename image `docker tag ghcr.io/autowarefoundation/autoware-universe:galactic-latest-prebuilt gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`
5. Upload image `docker push gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`

## Modifications to autoware.universe
### Disable trajectory planning of autoware

1. Comment out the planning part in the launch file: `~/workspace/autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml`
2. Run `colcon build --packages-select autoware_launch && source ~/autoware/install/setup.bash` in the autoware container.

## How to use
0. Create **2** terminals (maybe Terminator is usefull here)

1. Terminal **1**: open cr2autoware container 
   - `rocker --nvidia --x11 --volume $HOME/workspace/workspace:/root/workspace -- gitlab.lrz.de:5005/cps/dfg-car`
   - `ros2 launch cr2autoware test.launch.py`

2. Terminal **2**: open autoware.universe container 
   - `rocker --nvidia --x11 --user --volume $HOME/workspace/autoware:$HOME/autoware --volume $HOME/workspace/workspace:$HOME/workspace -- gitlab.lrz.de:5005/cps/dfg-car:autoware-universe`
   - `source ~/autoware/install/setup.bash && ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/workspace/dfg-car/src/cr2autoware/data/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit`

On this page is shown how to use the simple simulator to init state of the car and set a goal: https://autowarefoundation.github.io/autoware-documentation/latest/tutorials/ad-hoc-simulation/planning-simulation/