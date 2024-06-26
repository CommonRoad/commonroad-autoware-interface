from collections import defaultdict
from dataclasses import dataclass

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.traffic_light import TrafficLight, TrafficLightState, TrafficLightDirection, TrafficLightCycle, TrafficLightCycleElement
from commonroad.scenario.scenario import Scenario


# own code base
from data_eval_utils import convert_ros2_time_tuple_to_float
from global_timer import GlobalTimer

# typing
from typing import Tuple, List


@dataclass
class LightAtTime:
    traffic_light: TrafficLight
    traffic_ligth_id: int
    direction: TrafficLightDirection
    active: bool
    color: List[TrafficLightState]
    ros2_time: float
    time_idx: int
    temporal_distance_to_step: float



def add_traffic_lights(
        traffic_lights_over_time: List[Tuple[Tuple[int, int], List[TrafficLight]]],
        global_timer: GlobalTimer,
        scenario_path: str,
        save_path: str,
) -> None:

    # sort by traffic light
    dict_traffic_light_id_to_light_at_time = defaultdict(list)
    for _tuple in traffic_lights_over_time:
        time_s = convert_ros2_time_tuple_to_float(_tuple[0])
        for traffic_light in _tuple[1]:
            dict_traffic_light_id_to_light_at_time[traffic_light.traffic_light_id].append(
                LightAtTime(
                    traffic_light=traffic_light,
                    traffic_ligth_id=traffic_light.traffic_light_id,
                    direction=traffic_light.direction,
                    active=traffic_light.active,
                    color=traffic_light.color,
                    ros2_time=time_s,
                    time_idx=global_timer.find_closest_time_step(time_s),
                    temporal_distance_to_step=global_timer.get_distance_to_closest_time_step(time_s)
                )
            )

    # sort by time
    for _key, _val in dict_traffic_light_id_to_light_at_time.items():
        _val.sort(key=lambda x: x.ros2_time)


    # update the traffic lights contained in the scenario
    scenario, planning_problem_set = CommonRoadFileReader(
        filename=scenario_path
    ).open()

    for traffic_light_id, light_states in dict_traffic_light_id_to_light_at_time.items():
        update_traffic_light(
            light_states=light_states,
            scenario=scenario
        )

    # save scenario
    file_writer = CommonRoadFileWriter(scenario, planning_problem_set)
    file_writer.write_to_file(save_path, OverwriteExistingFile.ALWAYS)



def update_traffic_light(
        light_states: List[LightAtTime],
        scenario: Scenario
) -> None:
    """
    Creates dynamic obstacle from sorted list of states.
    :param states: list of ObstacleOverTime instances for one vehicle in ascending time order
    :param global_timer: global timer instance
    :return: dynamic obstacle
    """

    # 1. create
    # 2. subsample
    # 3. Add a cycle per step with current color


    # Filter out multiple entries for same step by taking the ones closest to the time steps continous time value
    dict_step_to_light_at_time = defaultdict(list)
    light_state_list: List[LightAtTime] = list()
    for light_state in light_states:
        dict_step_to_light_at_time[light_state.time_idx].append(light_state)
    for step, states in dict_step_to_light_at_time.items():
        light_state_list.append(min(states, key=lambda x: x.temporal_distance_to_step))

    # create traffic light cycle
    traffic_light_cycle_elements: List[TrafficLightCycleElement] = [
        TrafficLightCycleElement(
            state=light_at_time.color[0],
            duration=1
        )
        for light_at_time in light_state_list
    ]

    traffic_light_cycle: TrafficLightCycle = TrafficLightCycle(
        cycle_elements=traffic_light_cycle_elements,
        time_offset=light_state_list[0].time_idx,
        active=light_state_list[0].active
    )

    # find traffic light and udpate its cycle
    traffic_light: TrafficLight = scenario.lanelet_network.find_traffic_light_by_id(
        traffic_light_id=light_state_list[0].traffic_ligth_id
    )

    # TODO: what is the difference between traffic light active und trafficlightcycle active
    traffic_light.traffic_light_cycle = traffic_light_cycle
    traffic_light.color = [TrafficLightState.RED]










