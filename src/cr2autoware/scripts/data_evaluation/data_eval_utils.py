from typing import Union, Tuple

def convert_ros2_time_tuple_to_float(
        time_tuple: Union[Tuple[int, int], float]
) -> float:
    """
    Converts ros2 time (= tuple[sec, nanosec]) to one float sec value.
    :param time_tuple: time tuple, either directly convertible as [int, float], or from ros2 as [float, float]
    :return: time as float in seconds
    """
    if(isinstance(time_tuple, float)):
        time: float = time_tuple
    else:
        time: float = float(time_tuple[0]) + (float(time_tuple[1]) / 1e9)
    return time





