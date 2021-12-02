import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from lxml import etree
import os

from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from crdesigner.input_output.api import lanelet_to_commonroad



class Cr2Auto(Node):

    def __init__(self):
        super().__init__('cr2autoware')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.declare_parameter('map_osm_file', '')
        self.declare_parameter('left_driving', False)
        self.declare_parameter('adjacencies', False)

        self.build_scenario()        

    def build_scenario(self):
        self.map_filename = self.get_parameter('map_osm_file').get_parameter_value().string_value
        self.left_driving = self.get_parameter('left_driving').get_parameter_value()
        self.adjacencies = self.get_parameter('adjacencies').get_parameter_value()
        self.scenario = lanelet_to_commonroad(self.map_filename, proj="", left_driving=self.left_driving, adjacencies=self.adjacencies)

        # add ego vehicle

        ## save map
        # store converted file as CommonRoad scenario
        writer = CommonRoadFileWriter(
            scenario=self.scenario,
            planning_problem_set=PlanningProblemSet(),
            author="Sebastian Maierhofer",
            affiliation="Technical University of Munich",
            source="CommonRoad Scenario Designer",
            tags={Tag.URBAN},
        )
        writer.write_to_file("output/ZAM_Lanelet-1_1-T1.xml", OverwriteExistingFile.ALWAYS)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    cr2auto = Cr2Auto()

    rclpy.spin(cr2auto)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cr2auto.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
