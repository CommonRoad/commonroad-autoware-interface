import os
import pickle
# typing
from typing import List, Tuple

# own code base
from add_obstacles import add_dynamic_obstacles
from add_planning_problem import add_planning_problem
from commonroad.scenario.traffic_light import TrafficLight
from global_timer import GlobalTimer
from scripts.data_evaluation.add_ego import add_ego_vehicle
from scripts.data_evaluation.interpolate_obstacles import interpolate_obstacles


def main(data_dir_path: str,
         saving_dir_path: str,
         xml_path: str
         ) -> None:
    """
    Script for data evaluation using pkl files.
    :param data_dir_path: path to recorded pkl data
    :param saving_dir_path: path to save data evaluation to
    :param xml_path: path to commonroad xml
    """

    # sanity checks
    if not os.path.exists(data_dir_path) or not os.path.isabs(data_dir_path):
        raise FileNotFoundError(f'data dir: {data_dir_path} not found or not absolute')

    if not os.path.exists(saving_dir_path) or not os.path.isabs(saving_dir_path):
        raise FileNotFoundError(f'saving dir: {saving_dir_path} not found or not absolute')

    if not os.path.exists(xml_path) or not os.path.isabs(xml_path):
        raise FileNotFoundError(f'commonroad map xml: {xml_path} not found or not absolute')

    # Extract CommonRoad objects from pkl
    files: List = os.listdir(data_dir_path)
    for file_name in files:
        if "driven_trajectory_" in file_name:
            driven_trajectory_pkl = os.path.join(data_dir_path, file_name)
        elif "goal_pose_" in file_name:
            goal_pose_pkl = os.path.join(data_dir_path, file_name)
        elif "initial_pose3d_" in file_name:
            initial_pose_pkl = os.path.join(data_dir_path, file_name)
        elif "planned_trajectory_" in file_name:
            planned_trajectory_pkl = os.path.join(data_dir_path, file_name)
        elif "reference_trajectory_" in file_name:
            reference_trajectory_pkl = os.path.join(data_dir_path, file_name)
        elif "predicted_obstacles_" in file_name:
            predicted_obstacles_pkl = os.path.join(data_dir_path, file_name)
        elif "traffic_lights" in file_name:
            traffic_lights_pkl = os.path.join(data_dir_path, file_name)

    with open(goal_pose_pkl, 'rb') as f:
        goal_pose_data = pickle.load(f)

    with open(initial_pose_pkl, 'rb') as f:
        initial_pose3d_data = pickle.load(f)

    with open(planned_trajectory_pkl, 'rb') as f:
        planned_trajectory_data = pickle.load(f)

    with open(reference_trajectory_pkl, 'rb') as f:
        reference_trajectory_data = pickle.load(f)

    with open(driven_trajectory_pkl, 'rb') as f:
        driven_trajectory_data = pickle.load(f)

    with open(predicted_obstacles_pkl, 'rb') as f:
        predicted_obstacles = pickle.load(f)


    with open(traffic_lights_pkl, 'rb') as f:
        traffic_lights: List[Tuple[Tuple[int, int], List[TrafficLight]]] = pickle.load(f)


    global_timer: GlobalTimer = GlobalTimer(
        driven_trajectory=driven_trajectory_data[0],
        traffic_light_data=None,
        predicted_obstacles=predicted_obstacles,
        downsample_ms=100
    )


    # Add planning problem to scenario
    add_planning_problem(
        scenario_path=xml_path,
        save_path=os.path.join(saving_dir_path, "scenario.xml"),
        initial_state_of_pp=driven_trajectory_data[0].state_list[0],
        goal_state_of_pp=driven_trajectory_data[0].state_list[-1],
        goal_width=2,
        goal_length=6,
        global_timer=global_timer
    )

    # Add dynamic obstacles and save it
    add_dynamic_obstacles(
      dynamic_obstacles_per_time_step=predicted_obstacles,
      scenario_path=os.path.join(saving_dir_path, "scenario.xml"),
      save_path=os.path.join(saving_dir_path, "scenario_obstacles.xml"),
      global_timer=global_timer
    )

    add_ego_vehicle(
        driven_trajectory=driven_trajectory_data[0].state_list,
        scenario_path=os.path.join(saving_dir_path, "scenario_obstacles.xml"),
        save_path=os.path.join(saving_dir_path, "scenario_obstacles_ego.xml"),
        global_timer=global_timer
    )

    interpolate_obstacles(
        scenario_path=os.path.join(saving_dir_path, "scenario_obstacles_ego.xml"),
        save_path=os.path.join(saving_dir_path, "scenario_obstacles_ego_interpolated.xml"),
    )


if __name__ == "__main__":
    import matplotlib

    matplotlib.use("TkAgg")

    # data_path = "/home/lercher/tum/edgar/data/converted/2025-04-11_first_test_safe_dist/SafeDistance1"
    # save_path = "/home/lercher/tum/edgar/data/converted/2025-04-11_first_test_safe_dist/CommonRoad"

    # data_path = "/home/lercher/tum/edgar/data/converted/2025-04-11_second_test_safe_dist/SafeDistance2_2"
    # save_path = "/home/lercher/tum/edgar/data/converted/2025-04-11_second_test_safe_dist/CommonRoad"

    data_path = "/home/lercher/tum/edgar/data/converted/2025-04-11_left_turn/LeftTurn2"
    save_path = "/home/lercher/tum/edgar/data/converted/2025-04-11_left_turn/CommonRoad"

    map_path = "/home/lercher/tum/edgar/campus_sven/tum_campus_0_2_13_test_traffic_lights.xml"

    main(
        data_dir_path=data_path,
        saving_dir_path=save_path,
        xml_path=map_path,
    )

