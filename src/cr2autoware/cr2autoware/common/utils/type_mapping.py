# standard imports
from typing import Dict
from typing import List
from uuid import UUID as PyUUID

# third party
import numpy as np

# ROS msgs
from geometry_msgs.msg import Polygon as PolygonMsg
from unique_identifier_msgs.msg import UUID as UUIDMsg

# Autoware msgs
from autoware_auto_perception_msgs.msg import ObjectClassification  # type: ignore

# commonroad-io imports
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.geometry.shape import Shape, Rectangle, Circle, Polygon


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
) -> Shape:
    """
    Convert Autoware shape to CommonRoad shape.

    :param aw_shape_type: Autoware shape type of the obstacle
    :param width: width of the obstacle
    :param length: length of the obstacle
    :param footprint: a specification of a polygon
    :return: CommonRoad shape
    """
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
        return Polygon(vertices=footprint_2d)

    else:
        raise TypeError("Unsupported Autoware shape type: " + str(aw_shape_type))


def aw_to_cr_shape_updater(
    dynamic_obstacle: DynamicObstacle,
    width: float,
    length: float,
    footprint: PolygonMsg,
) -> None:
    """Update the shape of a CommonRoad dynamic obstacle.

    :param dynamic_obstacle: CommonRoad dynamic obstacle
    :param width: width of the obstacle
    :param length: length of the obstacle
    :param footprint: a specification of a polygon
    """
    shape = dynamic_obstacle.obstacle_shape
    
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
        dynamic_obstacle.obstacle_shape.vertices = footprint_2d

    else:
        raise TypeError("Unsupported CommonRoad shape type: " + str(dynamic_obstacle.obstacle_shape))


def uuid_from_ros_msg(
        msg: UUIDMsg
) -> PyUUID:
    """
    Converts a ROS UUID message to a Python type UUID.
    ROS UUID is represented as uint8[16], i.e., an array of length 16 containing 8-byte unsigned integers.

    :param msg: ROS UUID message of type unique_identifier_msgs/msg/UUID
    """
    return PyUUID(bytes=bytes([int(_elem) for _elem in msg]))
