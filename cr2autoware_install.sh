#! /bin/sh

# This is an automatic setup of cr2autoware when used in the TUM Autoware launch

echo "Starting Cr2Autoware Setup"
sudo apt-get update
sudo apt install -y python3-pykdl
cd ../commonroad-scenario-designer
pip install -r requirements.txt
cd ../commonroad-search
pip install -r requirements.txt
cd ../reactive_planner
pip install -r requirements.txt
pip install methodtools
pip install utm
pip install commonroad-io==2022.3
pip install commonroad-drivability-checker==2022.2.1
pip install commonroad-vehicle-models==3.0.0
pip install commonroad-route-planner==2022.3
pip install shapely==1.8.5
pip install -U numpy
echo "Cr2Autoware Setup completed!"
