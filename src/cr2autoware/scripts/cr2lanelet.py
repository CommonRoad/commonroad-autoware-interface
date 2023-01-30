import os
from commonroad.common.file_reader import CommonRoadFileReader
from lxml import etree
from crdesigner.map_conversion.lanelet_lanelet2.cr2lanelet_autoware import CR2LaneletConverter
import utm
import yaml

# ----------------------------------------------- Conversion ------------------------------------------------
# Commonroad to Lanelet2
# commonroad_to_lanelet(input_path, output_name, proj)

if False:
    simple_cr_scenario_path = "../../../ZAM_Lanelet-1_1-T1.xml"
    #                          "ZAM_Lanelet-1_1-T1.xml DEU_Backnang-7_4_T-1.xml DEU_Muc-4_1_T-1.xml"
    input_path = simple_cr_scenario_path
    output_name = "../../../test2.osm"
    proj = "+proj=utm +zone=32 +datum=WGS84 +ellps=WGS84"

    try:
        commonroad_reader = CommonRoadFileReader(input_path)
        scenario, _ = commonroad_reader.open()
    except etree.XMLSyntaxError as xml_error:
        print(f"SyntaxError: {xml_error}")
        print(
            "There was an error during the loading of the selected CommonRoad file.\n"
        )
        scenario = None

    if scenario:
        l2osm = CR2LaneletConverter(proj)
        osm = l2osm(scenario)
        with open(f"{output_name}", "wb") as file_out:
            file_out.write(
                etree.tostring(
                    osm, xml_declaration=True, encoding="UTF-8", pretty_print=True
                )
            )
        print(f"Conversion is successful, The output is at {os.path.abspath(output_name)}")
else:
    # simple_cr_scenario_path = "../../../DEU_Backnang-7_4_T-1.xml"
    # output_name = "../../../testJan.osm"
    basis_path = "/home/andrii/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/data/test_maps/lanelet2/"
    map_name = "merging_lanelets_utm"
    simple_cr_scenario_path = basis_path + map_name + "/" + map_name + ".xml"
    output_path = basis_path + map_name + "/"
    output_map_path = output_path + "lanelet2_map.osm"
    output_map_config_path = output_path + "map_config.yaml"

    try:
        commonroad_reader = CommonRoadFileReader(simple_cr_scenario_path)
        scenario, _ = commonroad_reader.open()
        lat, lon = float(scenario.location.gps_latitude), float(scenario.location.gps_longitude)
        if abs(lon) <= 180 and abs(lat) <= 90:
            proj = "+proj=utm +zone=%d +datum=WGS84 +ellps=WGS84" % utm.from_latlon(lat, lon)[2]
            print(utm.from_latlon(lat, lon)[2])
        else:
            proj = "+proj=utm +zone=32 +ellps=WGS84"
            # print(utm.from_latlon(11.66821, 48.26301))
            
    except etree.XMLSyntaxError as xml_error:
        print(f"SyntaxError: {xml_error}")
        print(
            "There was an error during the loading of the selected CommonRoad file.\n"
        )
        scenario = None

    use_local_coordinates = True
    l2osm = CR2LaneletConverter(proj, use_local_coordinates)
    osm = l2osm(scenario)
    with open(f"{output_map_path}", "wb") as file_out:
        file_out.write(
            etree.tostring(
                osm, xml_declaration=True, encoding="UTF-8", pretty_print=True
            )
        )

    data = {
        "/**" : {
            "ros__parameters": {
                "map_origin": {
                    "latitude": lat,
                    "longitude": lon,
                    "elevation": 0.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0 
                },
                "use_local_coordinates": use_local_coordinates
            }
        }
    }

    with open(output_map_config_path, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)
    
    print("output map path:", output_map_path)

