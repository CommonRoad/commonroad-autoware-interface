from lxml import etree
import os
import utm
import yaml

from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet

from crdesigner.map_conversion.lanelet2.lanelet2cr import Lanelet2CRConverter
from crdesigner.map_conversion.lanelet2.lanelet2_parser import Lanelet2Parser

from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad
from crdesigner.common.config.general_config import general_config
from crdesigner.common.config.lanelet2_config import lanelet2_config

DEFAULT_PROJ_STRING = "+proj=utm +zone=32 +ellps=WGS84"
basis_path = "/home/drivingsim/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/data/test_maps/lanelet2/"
map_name = "merging_lanelets_utm"
simple_cr_scenario_path = basis_path + map_name + "_from_osm.xml"
input_path = basis_path + map_name + "/"
input_map_path = input_path + "lanelet2_map.osm"
input_map_config_path = input_path + "map_config.yaml"

try:
    with open(input_map_config_path, 'r') as stream:
        data_loaded = yaml.safe_load(stream)
    print(data_loaded)
    map_origin = data_loaded["/**"]["ros__parameters"]["map_origin"]
    lat, lon = float(map_origin["latitude"]), float(map_origin["longitude"])
    proj = "+proj=utm +zone=%d +datum=WGS84 +ellps=WGS84" % utm.from_latlon(lat, lon)[2]
    print(utm.from_latlon(lat, lon)[2])
except FileNotFoundError:
    print("map config file not found:", input_map_config_path, "\nUsing default proj:", DEFAULT_PROJ_STRING)


left_driving = False  # replace with favoured value
adjacencies = False  # replace with favoured value

general_config.proj_string_cr = proj

lanelet2_config.left_driving = left_driving
lanelet2_config.adjacencies = adjacencies
lanelet2_config.translate = True
lanelet2_config.autoware = True

scenario = lanelet_to_commonroad(
    input_map_path,
    general_conf=general_config,
    lanelet2_conf=lanelet2_config,
)

scenario.convert_to_2d()

# store converted file as CommonRoad scenario
writer = CommonRoadFileWriter(
    scenario=scenario,
    planning_problem_set=PlanningProblemSet(),
    author="Sebastian Maierhofer",
    affiliation="Technical University of Munich",
    source="CommonRoad Scenario Designer",
    tags={Tag.URBAN},
)
writer.write_to_file(simple_cr_scenario_path, OverwriteExistingFile.ALWAYS)
