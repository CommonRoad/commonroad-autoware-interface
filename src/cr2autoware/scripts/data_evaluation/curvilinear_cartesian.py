import numpy as np
from shapely import Polygon
import math

# commonroad
import commonroad_dc.pycrccosy as pycrccosy

# Typing
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from typing import Tuple




def create_curvilinear_coordinate_system(reference_path: np.ndarray,
                                         limit_projection_domain: float = 30.0,
                                         eps: float = 0.1,
                                         eps2: float = 1e-4
                                         ) -> pycrccosy.CurvilinearCoordinateSystem:
    """
    Creates a curvilinear coordinate system from the given reference path.

    :param reference_path: reference path along which the clcs is created
    :param limit_projection_domain: default size limit for projection domain
    :param eps: To account for numerical imprecisions, eps reduces the computed lateral distance
    :param eps2: To account for numerical imprecisions, eps reduces the computed lateral distance

    :return: CommonRoad CLCS object
    """
    clcs = pycrccosy.CurvilinearCoordinateSystem(reference_path, limit_projection_domain, eps, eps2)
    clcs.compute_and_set_curvature()

    return clcs


def convert_point_curvillinear_to_cartesian(clcs: "CurvilinearCoordinateSystem",
                                            p_lon: float,
                                            p_lat: float
                                            ) -> np.ndarray:
    """
    Convert a point from vehicle-specific curvilinear coordinate system to global cartesian coordinate frame
    :param CLCS: from vehicle.config.planning.CLCS
    :param p_lon: longitudinal coordinate of point
    :param p_lat: lateral coordinate of the point

    :return: numpy array of point in cartesian x-y
    """
    point_converted: np.ndarray = clcs.convert_to_cartesian_coords(p_lon, p_lat)
    return point_converted


def convert_point_cartesian_to_curvilinear(clcs: "CurvilinearCoordinateSystem",
                                           x: float,
                                           y: float
                                           ) -> np.ndarray:
    """
    Convert a point from global cartesian coordinate frame to local curviliniear coordinate frame
    param: CLCS: curvilinear coordinate system
    param: x: x coordinate of point
    param: y: y coordinate of the point
    """
    point_converted: np.ndarray = clcs.convert_to_curvilinear_coords(x, y)
    return point_converted




def convert_velocity_vector_curvilinear_to_cartesian(clcs: "CurvilinearCoordinateSystem",
                                                     p_lon: float,
                                                     v_lon: float,
                                                     v_lat: float
                                                     ) -> np.ndarray:
    """
    Converts a velocity vector from curvilinear to cartesian using the angle of the tangent.

    :param clcs: curvilinear coordinate system
    :param p_lon: longitudinal position
    :param v_lon: longitudinal velocity
    :param v_lat: lateral velocity

    :return: cartesian velocity as numpy array vx_vy
    """
    tangent_cart = clcs.tangent(p_lon)

    alpha: float= math.atan2(tangent_cart[1], tangent_cart[0])

    v_x: float = math.cos(alpha) * v_lon - math.sin(alpha) * v_lat
    v_y: float = math.sin(alpha) * v_lon + math.cos(alpha) * v_lat

    return np.asarray([v_x, v_y])



def convert_velocity_vector_cartesian_to_curvilinear(clcs: "CurvilinearCoordinateSystem",
                                                     x: float,
                                                     y: float,
                                                     v_x: float,
                                                     v_y: float
                                                     ) -> np.ndarray:
    """
    Converts a velocity vector from cartesian to curvilinear using the angle of the tangent.

    :param clcs: curvilinear coordinate system
    :param x: cartesian position x
    :param y: cartesian position y
    :param v_x: cartesian velocity x
    :param v_y: cartesian velocity y

    :return: curvilinear velocity v_lon v_lat
    """

    # find corresponding p_lon
    point_converted = convert_point_cartesian_to_curvilinear(clcs, x, y)
    p_lon = point_converted[0]

    # compute forward angle alphay cart -> cvl
    tangent_cart = clcs.tangent(p_lon)
    alpha: float= math.atan2(tangent_cart[1], tangent_cart[0])

    # special cases to avoid dividing by zero:
    if(np.isclose(alpha, 0)):
        v_lon = v_x
        v_lat = 0
    elif(np.isclose(math.pi/2, 0)):
        v_lon = 0
        v_lat = v_x
    else:
    # normal case
        # use forward angle to compute backward
        v_lon: float = (v_x/math.cos(alpha)) + (v_y/math.cos(math.pi/2-alpha))
        v_lat: float = (-v_x/math.cos(math.pi/2-alpha)) + (v_y/math.cos(alpha))

    return np.asarray([v_lon, v_lat])



def convert_accelereration_curvilinear_to_cartesian(clcs: "CurvilinearCoordinateSystem",
                                                    p_lon: float,
                                                    a_lon: float,
                                                    a_lat: float
                                                    ) -> np.ndarray:
    """
    Convert acceleration from vehicle-specific curvilinear to global cartesian coordinate system.

    :param clcs: curvilinear coordinate system
    :param p_lon: longitudinal position
    :param a_lon: longitudinal acceleration
    :param a_lat: lateral acceleration

    :return: converted acceleration in cartesian a_x a_y
    """
    # same mathematical principle than for velocity
    return convert_velocity_vector_curvilinear_to_cartesian(clcs, p_lon, a_lon, a_lat)



def convert_accelleration_cartesian_to_curvilinear(clcs: "CurvilinearCoordinateSystem",
                                                   x: float,
                                                   y: float,
                                                   a_x: float,
                                                   a_y: float
                                                   ) -> np.ndarray:
    """
    Convert acceleration from global cartesian to vehicle-specific curvilinear coordinate system

    :param clcs: curvilinear coordinate system
    :param x: cartesian x position
    :param y: cartesian y position
    :param a_x: acceleration in x
    :param a_y: acceleration in y

    :return: curvilinear acceleration as numpy array a_lon a_lon
    """
    # same mathematical principle than for velocity
    return convert_velocity_vector_cartesian_to_curvilinear(clcs, x, y, a_x, a_y)









def convert_shapely_polygon_cartesian_to_curvilinear(clcs: "CurvilinearCoordinateSystem",
                                                     polygon: Polygon
                                                     ) -> Polygon:
    """
    Converts shapely polygon from cartesian to curvilinear

    :param clcs: curvilinear coordinate system
    :param polygon: shapely polygon object cartesian with exactly four vertices

    :return: shapely polygon object curvilinear
    """

    x_min: float = polygon.bounds[0]
    y_min: float = polygon.bounds[1]
    x_max: float = polygon.bounds[2]
    y_max: float = polygon.bounds[3]

    vertex1 = clcs.convert_to_curvilinear_coords(x_min, y_min)
    vertex2 = clcs.convert_to_curvilinear_coords(x_max, y_min)
    vertex3 = clcs.convert_to_curvilinear_coords(x_max, y_max)
    vertex4 = clcs.convert_to_curvilinear_coords(x_min, y_max)

    polygon_curv = Polygon([vertex1, vertex2, vertex3, vertex4])


    return polygon_curv



def convert_shapely_polygon_curvilinear_to_cartesian(clcs: "CurvilinearCoordinateSystem",
                                                     polygon: Polygon
                                                     ) -> Polygon:
    """
    Converts shapely polygon from curvilinear to cartesian

    :param clcs: curvilinear coordinate system
    :param polygon: shapely polygon object in curvilinear with exactly four vertices

    :return: shapely polygon object in cartesian
    """

    p_lon_min: float = polygon.bounds[0]
    p_lat_min: float = polygon.bounds[1]
    p_lon_max: float = polygon.bounds[2]
    p_lat_max: float = polygon.bounds[3]

    vertex1 = clcs.convert_to_cartesian_coords(p_lon_min, p_lat_min)
    vertex2 = clcs.convert_to_cartesian_coords(p_lon_max, p_lat_min)
    vertex3 = clcs.convert_to_cartesian_coords(p_lon_max, p_lat_max)
    vertex4 = clcs.convert_to_cartesian_coords(p_lon_min, p_lat_max)

    polygon_cart = Polygon([vertex1, vertex2, vertex3, vertex4])


    return polygon_cart


def get_orientation_cartesian_from_curvilinear(clcs: "CurvilinearCoordinateSystem",
                                               p_lon: float
                                               ) -> float:
    """
    Returns rad valued angle in cartesian from curvilinear point.
    Orientation approximated via tangent on p_lon axis.

    :param clcs: curvilinear coordiante system
    :param p_lon: longitudinal position

    :return: cartesian orientation in rad with respect to reference path
    """
    tangent_cart = clcs.tangent(p_lon)
    alpha: float = math.atan2(tangent_cart[1], tangent_cart[0])
    return alpha


def get_orientation_cartesian_from_cartesian(clcs: "CurvilinearCoordinateSystem",
                                             x: float,
                                             y: float
                                             ) -> float:
    """
    Returns rad valued angle in cartesian from cartesian point.
    Orientation approximated via tangent on p_lon axis.

    :param clcs: curvilinear coordiante system
    :param x: cartesian x position
    :param y: cartesian y position

    :return: orientation in cartesian in rad
    """

    # find corresponding p_lon
    point_converted = convert_point_cartesian_to_curvilinear(clcs, x, y)
    p_lon = point_converted[0]

    # compute forward angle alphay cart -> cvl
    tangent_cart = clcs.tangent(p_lon)
    alpha: float= math.atan2(tangent_cart[1], tangent_cart[0])
    return alpha


def get_velocity_cartesian_from_cartesian_orientation(v_total: float,
                                                      orientation_rad: float
                                                      ) -> Tuple[float, float]:
    """
    Takes total velocity and orientation(rad) with respect to the cartesian x-axis and returns v_x and v_y in cartesian.

    :param v_total: total velocity in cartesian
    :param orientation_rad: orientation cartesian in rad

    :return: tuple of v_x v_y in cartesian
    """
    v_x = math.acos(orientation_rad) * v_total
    v_y = math.asin(orientation_rad) * v_total
    return (v_x, v_y)