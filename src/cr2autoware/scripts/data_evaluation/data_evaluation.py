import enum
import os
import pickle
from dataclasses import dataclass
from functools import cached_property
from typing import List, Tuple, Optional

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer

from add_obstacles import add_dynamic_obstacles
from add_planning_problem import add_planning_problem
from global_timer import GlobalTimer
from scripts.data_evaluation.add_ego import add_ego_vehicle
from scripts.data_evaluation.interpolate_obstacles import interpolate_obstacles
from scripts.data_evaluation.visualization import draw_with_slider, get_ego_params


@enum.unique
class Location(enum.Enum):
    Lichtenberg = enum.auto()
    LichtenbergHansPiloty = enum.auto()
    LudwigPrandtlBoltzmann = enum.auto()

    @cached_property
    def plot_limits(self) -> List[int]:
        match self:
            case Location.Lichtenberg:
                return [470, 900, 680, 760]
            case Location.LichtenbergHansPiloty:
                return [500, 650, 660, 760]
            case Location.LudwigPrandtlBoltzmann:
                return [725, 940, 25, 100]
            case _:
                raise NotImplementedError("Unreachable")


@dataclass(frozen=True)
class Experiment:
    name: str
    location: Location
    movie_name: str = "commonroad.mp4"

    @cached_property
    def base_path(self) -> str:
        return os.path.join("/home/lercher/tum/edgar/data/artifact", self.name)

    @cached_property
    def data_path(self) -> str:
        return os.path.join(self.base_path, "Pickle")

    @cached_property
    def save_path(self) -> str:
        return os.path.join(self.base_path, "CommonRoad")

    @cached_property
    def movie_path(self) -> str:
        return os.path.join(self.base_path, self.movie_name)


def main(force_recreate: bool = False) -> None:
    base_path = "/home/lercher/tum/edgar/data/artifact"
    map_path = os.path.join(base_path, "tum_campus_2025-04-15.xml")
    experiments = [
        Experiment("2025-04-11_first_test_safe_dist", Location.Lichtenberg),
        Experiment("2025-04-11_second_test_safe_dist", Location.Lichtenberg),
        Experiment("2025-04-11_left_turn", Location.LichtenbergHansPiloty),
        Experiment("2025-04-16_first_test_safe_dist", Location.LudwigPrandtlBoltzmann),
        Experiment("2025-04-16_second_test_safe_dist", Location.LudwigPrandtlBoltzmann),
        Experiment("2025-04-16_right_turn", Location.LichtenbergHansPiloty),
    ]

    for experiment in experiments:
        if not os.path.exists(experiment.save_path):
            os.makedirs(experiment.save_path)
        scenario_path = os.path.join(
            experiment.save_path, "scenario_obstacles_ego_interpolated.xml"
        )
        if not os.path.exists(scenario_path) or force_recreate:
            create_commonroad_scenarios(
                data_dir_path=experiment.data_path,
                saving_dir_path=experiment.save_path,
                xml_path=map_path,
            )

        visualize(
            scenario_path,
            experiment.movie_path,
            figsize=(50, 40),
            plot_limits=experiment.location.plot_limits,
            show=True,
        )


def create_commonroad_scenarios(
    data_dir_path: str, saving_dir_path: str, xml_path: str
) -> None:
    """
    Script for data evaluation using pkl files.
    :param data_dir_path: path to recorded pkl data
    :param saving_dir_path: path to save data evaluation to
    :param xml_path: path to commonroad xml
    """

    # sanity checks
    if not os.path.exists(data_dir_path) or not os.path.isabs(data_dir_path):
        raise FileNotFoundError(f"data dir: {data_dir_path} not found or not absolute")

    if not os.path.exists(saving_dir_path) or not os.path.isabs(saving_dir_path):
        raise FileNotFoundError(
            f"saving dir: {saving_dir_path} not found or not absolute"
        )

    if not os.path.exists(xml_path) or not os.path.isabs(xml_path):
        raise FileNotFoundError(
            f"commonroad map xml: {xml_path} not found or not absolute"
        )

    # Extract CommonRoad objects from pkl
    files: List = os.listdir(data_dir_path)
    driven_trajectory_pkl: str = ""
    predicted_obstacles_pkl: str = ""
    for file_name in files:
        if "driven_trajectory_" in file_name:
            driven_trajectory_pkl = os.path.join(data_dir_path, file_name)
        elif "predicted_obstacles_" in file_name:
            predicted_obstacles_pkl = os.path.join(data_dir_path, file_name)
    if not driven_trajectory_pkl or not predicted_obstacles_pkl:
        raise FileNotFoundError("Could not find required pkl files in data dir")

    with open(driven_trajectory_pkl, "rb") as f:
        driven_trajectory_data = pickle.load(f)

    with open(predicted_obstacles_pkl, "rb") as f:
        predicted_obstacles = pickle.load(f)

    global_timer: GlobalTimer = GlobalTimer(
        driven_trajectory=driven_trajectory_data[0],
        traffic_light_data=None,
        predicted_obstacles=predicted_obstacles,
        downsample_ms=100,
    )

    # Add planning problem to scenario
    add_planning_problem(
        scenario_path=xml_path,
        save_path=os.path.join(saving_dir_path, "scenario.xml"),
        initial_state_of_pp=driven_trajectory_data[0].state_list[0],
        goal_state_of_pp=driven_trajectory_data[0].state_list[-1],
        goal_width=2,
        goal_length=6,
        global_timer=global_timer,
    )

    # Add dynamic obstacles and save it
    add_dynamic_obstacles(
        dynamic_obstacles_per_time_step=predicted_obstacles,
        scenario_path=os.path.join(saving_dir_path, "scenario.xml"),
        save_path=os.path.join(saving_dir_path, "scenario_obstacles.xml"),
        global_timer=global_timer,
    )

    add_ego_vehicle(
        driven_trajectory=driven_trajectory_data[0].state_list,
        scenario_path=os.path.join(saving_dir_path, "scenario_obstacles.xml"),
        save_path=os.path.join(saving_dir_path, "scenario_obstacles_ego.xml"),
        global_timer=global_timer,
    )

    interpolate_obstacles(
        scenario_path=os.path.join(saving_dir_path, "scenario_obstacles_ego.xml"),
        save_path=os.path.join(
            saving_dir_path, "scenario_obstacles_ego_interpolated.xml"
        ),
    )


def visualize(
    scenario_path: str,
    save_path: str,
    ego_id: int = 42,
    show: bool = False,
    figsize: Optional[Tuple[int, int]] = None,
    plot_limits: Optional[List[float]] = None,
    focus_ego: bool = False,
) -> None:
    """
    Visualizes the scenario and planning problem set.
    :param scenario_path: path to CommonRoad scenario file
    :param save_path: path to save the visualization
    :param ego_id: ID of the ego vehicle
    :param show: whether to show the visualization
    :param figsize: figure size for the visualization
    :param plot_limits: plot limits for the visualization
    :param focus_ego: whether to focus on the ego vehicle
    """

    # Load commonroad scenario
    scenario, planning_problem_set = CommonRoadFileReader(
        filename_2020a=scenario_path
    ).open()
    ego = scenario.obstacle_by_id(ego_id)
    scenario.remove_obstacle(ego)

    draw_params = MPDrawParams(
        time_begin=ego.initial_state.time_step, time_end=ego.prediction.final_time_step
    )
    draw_params.dynamic_obstacle.draw_icon = True

    if show:
        draw_with_slider(
            scenario,
            planning_problem_set,
            ego,
            draw_params=draw_params,
            figsize=figsize,
            plot_limits=plot_limits,
            focus_ego=focus_ego,
        )

    ego_params = get_ego_params(draw_params)
    rnd = MPRenderer(
        draw_params=draw_params,
        figsize=figsize,
        plot_limits=plot_limits,
        focus_obstacle=ego if focus_ego else None,
    )
    rnd.create_video([scenario, ego], save_path, draw_params=[draw_params, ego_params])


if __name__ == "__main__":
    import matplotlib

    matplotlib.use("TkAgg")

    main()
