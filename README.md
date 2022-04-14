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
| commonroad-io | 2020.3 |
| commonroad-drivability-checker | 2020.1 |
| commonroad-vehicle-models | 1.0.0 |
| commonroad-search | branch: master |
| autoware.auto |commit: fd8b3e27000837b8e4bd9417ece4367823d468a5|

## How to start

0. to run the docker image, open a terminal and enter under `~/adehome/Autoware`:
   - `ade --rc amd64-foxy-lgsvl start`
then use [terminator](https://wiki.ubuntuusers.de/Terminator/) to open 4 terminals for the following steps, each run:
   - `ade enter`

(note: if you reboot the laptop or use `ade stop` to exist the ade environment, you have to use `ade --rc amd64-foxy-lgsvl start` to enter the image. If you just log out, you don't have to start the ade image again.)

1. compile **cr2autoware** package from source.
   * source /opt/AutowareAuto/setup.bash
   * navigate to workspace/dfg-car directory and run **colcon build**
- Terminal 1 (under workspace/dfg-car):
  - `source ./install/setup.bash`
  - `ros2 launch cr2autoware autoware_auto_visualization.launch.py`
- Terminal 2:
  - `source ./install/setup.bash`
  - `ros2 launch cr2autoware avp_sim.launch.py`

2. run SVL simulator:
   - `/opt/lgsvl/simulator &`
   - open browser, logging in with tuminfocps@gmail.com with password ilovecps, click on `run simulator`
   - SVLSimulator is then loaded, click on the start button
   - Initializing the localization, see [here](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/ndt-initialization.html)
3. run cr2autoware planner node in another terminal:
   - `sudo apt-get update`
   - `sudo apt-get install python3-pykdl`
   - `source ./install/setup.bash`
   - `ros2 launch cr2autoware test.launch.py` 

Video:  /home/drivingsim/Videos/how_to_run.webm
