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
