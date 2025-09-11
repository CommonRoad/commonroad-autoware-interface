from typing import Any
import copy
import numpy as np
from commonroad.scenario.lanelet import Lanelet
from dataclasses import dataclass

def copy_from_blackboard(blackboard_param: Any) -> Any:
        """
        Deep copy the parameter from the blackboard.

        :param blackboard_param: Parameter from the blackboard
        :return: Deep copy of the parameter
        """
        return copy.deepcopy(blackboard_param)


def calculate_current_position_index(current_position_curvilinear: np.ndarray, input_path_curvilinear: np.ndarray) -> int:
        """
        Calculate the index of the current position in the input path.

        For calculation, the curvilinear coordinates are used.

        :param current_position_curvilinear: Current position in curvilinear coordinates
        :param input_path_curvilinear: Input path in curvilinear coordinates
        :return: Index of the current position in the input path
        """
        distances = np.abs(input_path_curvilinear - current_position_curvilinear[0])
        return np.argmin(distances)


def minimum_width_lanelet(lanelet: Lanelet) -> float:
    """
    Calculate the minimum width of the lanelet by finding the minimum distance between left and right vertices.

    :param lanelet: lanelet of a CommonRoad scenario
    :return: The minimum width of the lanelet.
    """
    left_vertices = lanelet.left_vertices
    right_vertices = lanelet.right_vertices
    widths = np.linalg.norm(left_vertices - right_vertices, axis=1)
    return np.min(widths)


@dataclass
class BehaviorScenarioParams:
        """
        Class to hold behavior scenario parameters.
        """
        cr_obstacle_box_front: float = None
        cr_obstacle_box_rear: float = None
        cr_obstacle_box_side: float = None
        cr_obstacle_box_prediction: bool = None
