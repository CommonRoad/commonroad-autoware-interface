"""
Type Mapping Utils
===========================

This module provides type mapping utilities for the CommonRoad to Autoware interface.

---------------------------
"""
# standard imports
from typing import Dict, List
from uuid import UUID as PyUUID

# third party
import numpy as np
from shapely.geometry import Polygon as PolygonShapely

# Autoware msgs
from autoware_auto_perception_msgs.msg import ObjectClassification  # type: ignore
from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape

# commonroad-io imports
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.traffic_light import TrafficLightDirection, TrafficLightState, TrafficLightCycle, TrafficLightCycleElement

# ROS msgs
from geometry_msgs.msg import Polygon as PolygonMsg
from unique_identifier_msgs.msg import UUID as UUIDMsg


# Dictionary to map Autoware classification to CommonRoad obstacle type
aw_to_cr_obstacle_type: Dict[int, ObstacleType] = {
    0: ObstacleType.UNKNOWN,
    1: ObstacleType.CAR,
    2: ObstacleType.TRUCK,
    3: ObstacleType.BUS,
    4: ObstacleType.TRUCK,
    5: ObstacleType.MOTORCYCLE,
    6: ObstacleType.BICYCLE,
    7: ObstacleType.PEDESTRIAN,
}


aw_to_cr_traffic_light_color: Dict[int, TrafficLightState] = {
    1: TrafficLightState.RED,  # AW: RED
    2: TrafficLightState.YELLOW,  # AW: AMBER
    3: TrafficLightState.GREEN,  # AW: GREEN

    99: TrafficLightState.INACTIVE,  # no AW state: represents a detected traffic light that is not active

    # additional states Autoware:
    # 18: UNKNOWN,
    # 4: WHITE,
    # additional states CommonRoad:
    # TrafficLightState.RED_YELLOW
    # TrafficLightState.INACTIVE
}


aw_to_cr_traffic_traffic_light_shape: Dict[int, TrafficLightDirection] = {
    5: TrafficLightDirection.ALL,  # AW: CIRCLE
    6: TrafficLightDirection.LEFT,  # AW: LEFT_ARROW
    7: TrafficLightDirection.RIGHT,  # AW: RIGHT_ARROW
    8: TrafficLightDirection.STRAIGHT,  # AW: UP_ARROW
    9: TrafficLightDirection.LEFT_STRAIGHT,  # AW: UP_LEFT_ARROW
    10: TrafficLightDirection.STRAIGHT_RIGHT,  # AW: UP_RIGHT_ARROW

    # additional states Autoware:
    # 11: DOWN_ARROW
    # 12: DOWN_LEFT_ARROW
    # 13: DOWN_RIGHT_ARROW
    # 18: UNKNOWN,
    # 0: CROSS,
    # additional states CommonRoad:
    # TrafficLightDirection.LEFT_RIGHT
}

aw_to_cr_traffic_traffic_light_status: Dict[int, bool] = {
    15: False,  # AW: SOLID_OFF
    16: True,  # AW: SOLID_ON

    # additional states Autoware:
    # 17: FLASHING
    # 18: UNKNOWN
}


def get_classification_with_highest_probability(
        classification: List[ObjectClassification]
) -> int:
    """Finds class with highest probatility.

    :param classification: list of object classifications
    :return: class with highest probability
    """
    return sorted(classification, key=lambda x: x.probability, reverse=True)[0].label


def aw_to_cr_shape(
        aw_shape_type: int,
        width: float,
        length: float,
        footprint: PolygonMsg,
        safety_margin: float,
) -> Shape:
    """
    Convert Autoware shape to CommonRoad shape.

    :param aw_shape_type: Autoware shape type of the obstacle
    :param width: width of the obstacle
    :param length: length of the obstacle
    :param footprint: a specification of a polygon
    :param safety_margin: safety margin for the obstacle
    :return: CommonRoad shape
    """
    # add safety margin
    width = 2 * safety_margin + width
    length = 2 * safety_margin + length

    if aw_shape_type == 0:
        assert width > 0.0 and length > 0.0, "Obstacle shape: Width and length must be positive."
        return Rectangle(width=width, length=length)

    elif aw_shape_type == 1:
        assert width > 0.0, "Obstacle shape: Width must be positive."
        return Circle(radius=(width / 2))

    elif aw_shape_type == 2:
        # convert 3D polygon footprint to 2D
        points = footprint.points
        assert points, "Obstacle shape: Footprint must be provided."
        footprint_2d = np.array([[point.x, point.y] for point in points])

        # add safety margin buffer for polygon
        polygon = PolygonShapely(footprint_2d)
        polygon_buffer = polygon.buffer(safety_margin, join_style="mitre")
        buffered_vertices = List(polygon_buffer.exterior.coords)
        footprint = [[x, y] for x, y in buffered_vertices]
        return Polygon(vertices=footprint_2d)

    else:
        raise TypeError("Unsupported Autoware shape type: " + str(aw_shape_type))


def aw_to_cr_shape_updater(
        dynamic_obstacle: DynamicObstacle,
        width: float,
        length: float,
        footprint: PolygonMsg,
        safety_margin: float,
) -> None:
    """Update the shape of a CommonRoad dynamic obstacle.

    :param dynamic_obstacle: CommonRoad dynamic obstacle
    :param width: width of the obstacle
    :param length: length of the obstacle
    :param footprint: a specification of a polygon
    :param safety_margin: safety margin for the obstacle
    """
    shape = dynamic_obstacle.obstacle_shape

    # add safety margin
    width = 2 * safety_margin + width
    length = 2 * safety_margin + length

    if isinstance(shape, Rectangle):
        assert width > 0.0 and length > 0.0, "Update obstacle shape: Width and length must be positive."
        dynamic_obstacle.obstacle_shape.width = width
        dynamic_obstacle.obstacle_shape.length = length

    elif isinstance(shape, Circle):
        assert width > 0.0, "Update obstacle shape: Width must be positive."
        dynamic_obstacle.obstacle_shape.radius = width / 2

    elif isinstance(shape, Polygon):
        # convert 3D polygon footprint to 2D
        points = footprint.points
        assert points, "Update obstacle shape: Footprint must be provided."
        footprint_2d = np.array([[point.x, point.y] for point in points])

        # add safety margin buffer for polygon
        polygon = PolygonShapely(footprint_2d)
        polygon_buffer = polygon.buffer(safety_margin, join_style="mitre")
        buffered_vertices = List(polygon_buffer.exterior.coords)
        footprint = [[x, y] for x, y in buffered_vertices]
        dynamic_obstacle.obstacle_shape.vertices = footprint_2d

    else:
        raise TypeError("Unsupported CommonRoad shape type: " + str(dynamic_obstacle.obstacle_shape))


def set_traffic_light_cycle(traffic_light_state: TrafficLightState) -> TrafficLightCycle:
    """
    Set the traffic light cycle based on the traffic light state.

    :param traffic_light_state: traffic light state
    :return: traffic light cycle
    """

    cycle_element = TrafficLightCycleElement(traffic_light_state, 5)

    traffic_light_cylce = TrafficLightCycle(cycle_elements=[cycle_element])

    return traffic_light_cylce


def uuid_from_ros_msg(
        msg: UUIDMsg
) -> PyUUID:
    """
    Converts a ROS UUID message to a Python type UUID.

    ROS UUID is represented as uint8[16], i.e., an array of length 16 containing 8-byte unsigned integers.

    :param msg: ROS UUID message of type unique_identifier_msgs/msg/UUID
    :return:
    """
    return PyUUID(bytes=bytes([int(_elem) for _elem in msg]))
