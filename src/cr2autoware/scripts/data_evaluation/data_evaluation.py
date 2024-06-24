import os
import sys
import pickle

# own code base
from visualization import visualize_route_and_trajectories
from physics_evaluation import visualize_planned_vs_driven_trajectory
from add_obstacles import add_dynamic_obstacles
from add_planning_problem import add_planning_problem

# typing
from typing import List


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


    # Add planning problem to scenario
    add_planning_problem(
        scenario_path=xml_path,
        save_path=os.path.join(saving_dir_path, "mit_planing_problem.xml"),
        initial_state_of_pp=planned_trajectory_data[0].state_list[0],
        goal_state_of_pp=goal_pose_data[0],
        goal_width=2,
        goal_length=6
    )

    # Add dynamic obstacles and save it
    add_dynamic_obstacles(
      dynamic_obstacles_per_time_step=predicted_obstacles,
      scenario_path=os.path.join(saving_dir_path, "mit_planing_problem.xml"),
      save_path=os.path.join(saving_dir_path, "scenario_with_pp_and_obstacles.xml")
    )

    # visualization parameters
    title_font_size: float = 50
    axis_font_size: float = 40
    plot_font_size: float = 30
    step: int = 151
    line_width: int = 3

    # Visualizes route with velocity profile
    visualize_route_and_trajectories(
        scenario_path=os.path.join(saving_dir_path, "scenario_with_pp_and_obstacles.xml"),
        save_path=os.path.join(saving_dir_path, "route.png"),
        save_img=True,
        draw_footprint=True,
        draw_ego_trajectory=False,
        draw_reference_trajectory=False,
        reference_trajectory=reference_trajectory_data[0].state_list,
        step=step,
        driven_trajectory=driven_trajectory_data[0].state_list,
    )

    # Visualizes planned vs driven trajectory
    visualize_planned_vs_driven_trajectory(
        planned_trajectories=planned_trajectory_data,
        driven_trajectory=driven_trajectory_data[0],
        reference_trajectory=reference_trajectory_data[0],
        save_img=True,
        save_path=saving_dir_path,
        linewidth=line_width,
        axis_font_size=axis_font_size,
        title_font_size=title_font_size,
        plot_font_size=plot_font_size
    )


if __name__ == "__main__":
    sim_folder: str = "/home/tmasc/Desktop/edgar_fahrten/sim/mit_auto"
    sim_save: str = "/home/tmasc/Desktop/edgar_fahrten/sim/mit_auto/auswertung_sim"

    # real_folder: str = "/home/tmasc/Desktop/edgar_fahrten/mcap_sim/asdf"
    # real_save: str = "/home/tmasc/Desktop/edgar_fahrten/mcap_sim/auswertung"

    real_folder: str = "/home/gerald/Documents/Research_Projects/EDGAR_MCube/05_Test_Drive_Data/2024_01_27_Test_Drives_CR2AW_Paper/PlanningSim/mit_auto/2024_01_21_14_26"
    real_save: str = "/home/gerald/Documents/Research_Projects/EDGAR_MCube/05_Test_Drive_Data/2024_01_27_Test_Drives_CR2AW_Paper/PlanningSim/mit_auto/2024_01_21_14_26/eval"

    xml_path = os.path.join(sim_folder, "2024_01_21_14_26.xml")

    main(
        data_dir_path=sim_folder,
        saving_dir_path=sim_folder,
        xml_path=xml_path
    )
