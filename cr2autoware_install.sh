#! /bin/sh

# This is setup script of cr2autoware to automatically fetch submodules and install pip dependencies

# Constants
GREEN='\033[0;32m'
NC='\033[0m'

function print_progress() {
  print "${GREEN}${1}${NC}" "${2}"
}

print_progress "<CR2Autoware>: Starting Cr2Autoware Setup" -n

# initialize and update submodules
# TODO: remove if dependencies are fetched via vcs
print_progress "<CR2Autoware>: Fetching submodules..." -n
git submodule init
git submdule update
print_progress "Done" -n

# install apt packages
print_progress "<CR2Autoware>: Installing apt packages..." -n
# TODO: include apt dependencies in dockerfile?
sudo apt-get update -y
sudo apt install -y python3-pykdl
# TODO Bugfix: missing in Dockerimage
sudo apt-get install -y liblttng-ust-dev
print_progress "Done" -n

# install pip packages
print_progress "<CR2Autoware>: Installing pip packages..." -n

cd dfg-car
pip install -r src/cr2autoware/requirements.txt

cd ../commonroad-scenario-designer
pip install .

cd ../commonroad-search
pip install .

cd ../reactive_planner
pip install .

# TODO: Bugfix numpy version
pip install -U numpy

print_progress "<CR2Autoware>: Cr2Autoware Setup completed!" -n
