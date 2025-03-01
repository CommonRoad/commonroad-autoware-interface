from typing import Any
import copy

def copy_from_blackboard(blackboard_param: Any) -> Any:
        """
        Deep copy the parameter from the blackboard.

        :param blackboard_param: Parameter from the blackboard
        :return: Deep copy of the parameter
        """
        return copy.deepcopy(blackboard_param)