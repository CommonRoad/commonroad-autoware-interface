import copy
from typing import Tuple, List

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider


def main():
    # sc, pp = CommonRoadFileReader("scenarios/tum_campus_0_2_13_test_traffic_lights.xml").open()

    # sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_first_test_safe_dist/CommonRoad/scenario_with_ego_interpolated.xml").open()
    # sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_second_test_safe_dist/CommonRoad/scenario_with_ego_interpolated.xml").open()
    sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_left_turn/CommonRoad/scenario_with_ego_interpolated.xml").open()

    # sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_first_test_safe_dist/CommonRoad/scenario_with_ego.xml").open()
    # sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_second_test_safe_dist/CommonRoad/scenario_with_ego.xml").open()
    # sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_left_turn/CommonRoad/scenario_with_ego.xml").open()

    ego = sc.obstacle_by_id(42)
    sc.remove_obstacle(ego)

    draw_params = MPDrawParams(time_begin=ego.initial_state.time_step, time_end=ego.prediction.final_time_step)
    draw_params.dynamic_obstacle.draw_icon = True
    figsize = (25, 20)
    plot_limits = [470, 900, 650, 770]
    # rnd = MPRenderer(draw_params=draw_params, figsize=(25, 20), plot_limits=[470, 900, 680, 770])  # safe dist 1
    # rnd = MPRenderer(draw_params=draw_params, figsize=(25, 20), plot_limits=[470, 825, 690, 770])  # safe dist 2
    # rnd = MPRenderer(draw_params=draw_params, figsize=(25, 20), plot_limits=[400, 900, 650, 780])  # left turn

    draw_with_slider(sc, pp, ego, draw_params, figsize=figsize, plot_limits=plot_limits)

    rnd = MPRenderer(draw_params=draw_params, figsize=figsize, plot_limits=plot_limits)
    ego_params = get_ego_params(draw_params)
    rnd.create_video([sc, ego], "scenario_with_ego.gif", draw_params=[draw_params, ego_params])

def draw_with_slider(scenario: Scenario, planning_problems: PlanningProblemSet, ego: DynamicObstacle, draw_params: MPDrawParams, figsize: Tuple[int, int] = None, plot_limits: List[float] = None):
    fig, ax = plt.subplots()
    step_slider = Slider(
        fig.add_axes([0.2, 0.1, 0.65, 0.03]),
        "Step",
        draw_params.time_begin,
        draw_params.time_end,
        valinit=draw_params.time_begin,
        valstep=1,
        initcolor="none",
    )
    if figsize:
        fig.set_size_inches(*figsize)



    def update(val):
        step_params = copy.deepcopy(draw_params)
        step_params.time_begin = int(step_slider.val)
        step_params.time_end = int(step_slider.val)
        rnd = MPRenderer(draw_params=step_params, plot_limits=plot_limits, ax=ax)
        scenario.draw(rnd)
        planning_problems.draw(rnd)
        ego_params = get_ego_params(step_params)
        ego.draw(rnd, draw_params=ego_params)
        rnd.render()

    update(step_slider.val)
    step_slider.on_changed(update)
    plt.show()


def get_ego_params(draw_params: MPDrawParams) -> MPDrawParams:
    ego_params = copy.deepcopy(draw_params)
    ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
    return ego_params

if __name__ == "__main__":
    import matplotlib

    matplotlib.use("TkAgg")
    main()