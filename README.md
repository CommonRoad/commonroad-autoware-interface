# Our DFG-Car: **EDGAR**

## Description
This project builds an interface between commonroad and autoware.auto. 

## Introduction of files
cr2autoware
* cr2autoware.py: a ros node that subscribe information from autoware and process information and publish data to autoware
* tf2_geometry_msgs.py: define help-functions for transformation between map frame and other frames.
* utils.py: used for visualization of planning

config
* avp.rviz: configuration for rviz and it defines the name of topic for goal pose: goal_pose_cr.

data:
* autonomoustuff_parking_lot.osm: map for avp.

param:
* cr2autoware_param_file.param.yaml: parameter for initialization of map and vehicle

## Build up environment
### Required tools and versions:
| Tools | Versions|
|-|-|
| commonroad-io | 2021.3 |
| commonroad-drivability-checker | 2021.4 |
| commonroad-vehicle-models | 2.0.0 |
| commonroad-search | branch: master |
| autoware.auto |commit: fd8b3e27000837b8e4bd9417ece4367823d468a5|


## Installation of Autoware 
See [Documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation.html). The recommended method for installation is through the use of [ADE](https://ade-cli.readthedocs.io/en/latest/), a Docker-based tool to ensure that all developers in a project have a common, consistent development environment. An exemplar installation guidance in the lab PC can be found [here](https://gitlab.lrz.de/cps/dfg-car/-/wikis/Installation-of-Autoware.Auto-in-the-lab-PC),

## Initiallization/Editting the source code
1. Open **pycharm** within ade:
   - `ade enter` (make sure you have already started ade)
   - navigate to `~/pycharm/pycharm-community-2021.3.1/bin`
   - enter `./pycharm.sh`
2. git pull and edit the source code
   - \* for the first time: creat `workspace` folder
3. compile **cr2autoware** package from source.
   - `source /opt/AutowareAuto/setup.bash`
   - navigate to `workspace/dfg-car` directory
   - run `colcon build`
4. git push

## How to use

0. to run the docker image, open a terminal and enter under `~/adehome/Autoware`:
   - `ade --rc .aderc-amd64-foxy-lgsvl start -f`
then use [terminator](https://wiki.ubuntuusers.de/Terminator/) to open 4 terminals for the following steps, each run:
   - `ade enter`
   - navigate to `workspace/dfg-car` 
(note: if you reboot the laptop or use `ade stop` to exist the ade environment, you have to use `ade --rc .aderc-amd64-foxy-lgsvl start -f` to enter the image. If you just log out, you don't have to start the ade image again.)

1. Terminal **1**: open rviz 
   - `source ./install/setup.bash`
   - `ros2 launch cr2autoware autoware_auto_visualization.launch.py`

2. Terminal **2**: launch avp 
   - `source ./install/setup.bash`
   - `ros2 launch cr2autoware avp_sim.launch.py`

3. Terminal **3**: run SVL simulator:
   - `/opt/lgsvl/simulator &`
   - open browser, logging in with tuminfocps@gmail.com with password ilovecps, click on `run simulator`
   - SVLSimulator is then loaded, click on the start button
   - Initializing the localization, see [here](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/ndt-initialization.html)

4. Terminal **4**: run cr2autoware planner node in another terminal:
   - `sudo apt-get update`
   - `sudo apt-get install python3-pykdl`
   - `source ./install/setup.bash`
   - `ros2 launch cr2autoware test.launch.py` 

Video:  see documents/how_to_run.webm

