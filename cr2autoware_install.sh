#! /bin/sh

# This is setup script of cr2autoware to automatically fetch submodules and install pip dependencies

# Constants
GREEN='\033[0;32m'
NC='\033[0m'


function print() {
  echo -e "${1}"
  if [ "${2}" == "-n" ]; then
    echo -e ""
  fi
}


function print_progress() {
  print "${GREEN}${1}${NC}" "${2}"
}

print_progress "<CR2Autoware>: Starting Cr2Autoware Setup" -n

# install required apt packages
print_progress "<CR2Autoware>: Installing apt packages..." -n
sudo apt-get update -y
sudo apt install -y python3-pykdl
# missing in Dockerimage
sudo apt-get install -y liblttng-ust-dev
# Rosbag writer for file writer
sudo apt-get install -y ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap
print_progress "Done" -n

# install pip packages
print_progress "<CR2Autoware>: Installing pip packages..." -n

cd dfg-car
print_progress "<CR2Autoware>: Installing dfg-car dependencies..." -n
pip install -r src/cr2autoware/requirements.txt
print_progress "Done" -n

cd ../commonroad-scenario-designer
print_progress "<CR2Autoware>: Installing commonroad-scenario-designer..." -n
pip install .
print_progress "Done" -n

cd ../reactive-planner
print_progress "<CR2Autoware>: Installing reactive-planner..." -n
pip install .
print_progress "Done" -n

cd ..

print_progress "<CR2Autoware>: Cr2Autoware Setup completed!" -n
