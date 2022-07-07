import os
from commonroad.common.file_reader import CommonRoadFileReader

from lxml import etree
from crdesigner.map_conversion.lanelet_lanelet2.cr2lanelet import CR2LaneletConverter

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
    simple_cr_scenario_path = "../../../DEU_Backnang-7_4_T-1.xml"
    output_name = "../../../testJan.osm"
    proj = "+proj=utm +zone=32 +datum=WGS84 +ellps=WGS84"

    try:
        commonroad_reader = CommonRoadFileReader(simple_cr_scenario_path)
        scenario, _ = commonroad_reader.open()

    except etree.XMLSyntaxError as xml_error:
        print(f"SyntaxError: {xml_error}")
        print(
            "There was an error during the loading of the selected CommonRoad file.\n"
        )
        scenario = None

    l2osm = CR2LaneletConverter(proj)
    osm = l2osm(scenario)
    with open(f"{output_name}", "wb") as file_out:
        file_out.write(
            etree.tostring(
                osm, xml_declaration=True, encoding="UTF-8", pretty_print=True
            )
        )
