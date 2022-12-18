#! /bin/sh

# This is an automatic setup of cr2autoware when used in the TUM Autoware launch
# place this file in the autoware folder

echo "Starting Cr2Autoware Setup"
sudo apt-get update
sudo apt install -y python3-pykdl
cd src/universe/autoware.universe/planning/tum_commonroad_planning/commonroad-scenario-designer
pip install -r requirements.txt
cd ../commonroad-search
pip install -r requirements.txt
cd ../reactive_planner
pip install -r requirements.txt
pip install methodtools
pip install utm
pip install commonroad-io==2022.1
pip install commonroad-drivability-checker==2021.4
pip install commonroad-vehicle-models==2.0.0
pip install commonroad-route-planner==2022.1
echo "Cr2Autoware Setup completed!"
