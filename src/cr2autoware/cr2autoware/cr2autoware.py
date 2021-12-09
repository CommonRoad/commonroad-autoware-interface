from numpy.lib.utils import source
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from lxml import etree

import numpy as np
import matplotlib.pyplot as plt
# import necessary classes from different modules
from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from commonroad.visualization.mp_renderer import MPRenderer

from crdesigner.input_output.api import lanelet_to_commonroad

from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_auto_perception_msgs.msg import BoundingBoxArray
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Cr2Auto(Node):

    def __init__(self):
        super().__init__('cr2autoware')
        self.declare_parameter('map_osm_file', '')
        self.declare_parameter('left_driving', False)
        self.declare_parameter('adjacencies', False)

        self.tf_buffer = Buffer()
        self.tf_listner = TransformListener(self.tf_buffer, self)               # convert among frames
        
        self.build_scenario()  
        """
        self.static_obs_subscriber = self.create_subscription(                  # static obstacles
            BoundingBoxArray,
            '/perception/lidar_bounding_boxes_filtered',
            self.static_obstacle_callback,
            10
        )
        """
        
        self.initialPose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/initialpose',
            self.initial_pose_callback,
            10
        )      

    def build_scenario(self):
        self.map_filename = self.get_parameter('map_osm_file').get_parameter_value().string_value
        self.left_driving = self.get_parameter('left_driving').get_parameter_value()
        self.adjacencies = self.get_parameter('adjacencies').get_parameter_value()
        self.scenario = lanelet_to_commonroad(self.map_filename,
                                              proj="", 
                                              left_driving=self.left_driving, 
                                              adjacencies=self.adjacencies)
        # add ego vehicle
        # add static obstacles
        # add dynamic obstacles
        # add problemset
        ## save map
        self.write_scenario()

    def initial_pose_callback(self, initial_pose: PoseWithCovarianceStamped):
        self.get_logger().info('Subscribing initial pose ...')
        now = rclpy.time.Time()
        self.transform = self.tf_buffer.lookup_transform("map", "earth", now, timeout=Duration(seconds=1.0))

        x = self.transform.transform.translation.x + initial_pose.pose.pose.position.x
        y = self.transform.transform.translation.y + initial_pose.pose.pose.position.y

        self.get_logger().info('x: %f, y: %f' % (x, y))

        # generate the static obstacle according to the specification, refer to API for details of input parameters
        static_obstacle_id = self.scenario.generate_object_id()
        static_obstacle_type = ObstacleType.PARKED_VEHICLE
        static_obstacle_shape = Rectangle(width = 2.0, length = 4.5)
        static_obstacle_initial_state = State(position = np.array([x, y]), orientation = 0.02, time_step = 0)

        # feed in the required components to construct a static obstacle
        ego_vehicle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape, static_obstacle_initial_state)

        # add the static obstacle to the scenario
        self.scenario.add_objects(ego_vehicle)

        self.write_scenario()

    def static_obstacle_callback(self, boxArray: BoundingBoxArray):
        self.get_logger().info('Subscribing static obstacles ...')

    def plot_scenario(self):
        plt.figure(figsize=(10, 10))
        rnd = MPRenderer()
        self.scenario.draw(rnd)
        rnd.render()

    def write_scenario(self):
        ## save map
        # store converted file as CommonRoad scenario
        writer = CommonRoadFileWriter(
            scenario=self.scenario,
            planning_problem_set=PlanningProblemSet(),
            author="",
            affiliation="Technical University of Munich",
            source="",
            tags={Tag.URBAN},
        )
        writer.write_to_file("output/ZAM_Lanelet-1_1-T1.xml", OverwriteExistingFile.ALWAYS)


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
