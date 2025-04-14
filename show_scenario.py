import copy

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt


def main():
    # sc, pp = CommonRoadFileReader("scenarios/tum_campus_0_2_13_test_traffic_lights.xml").open()
    # sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_second_test_safe_dist/CommonRoad/scenario_with_pp_and_obstacles.xml").open()
    sc, pp = CommonRoadFileReader("/home/lercher/tum/edgar/data/converted/2025-04-11_left_turn/CommonRoad/scenario_with_ego.xml").open()
    ego = sc.obstacle_by_id(42)
    draw_params = MPDrawParams(time_begin=ego.initial_state.time_step, time_end=ego.prediction.final_time_step)
    draw_params.dynamic_obstacle.draw_icon = True
    # draw_params.lanelet_network.lanelet.show_label = True
    rnd = MPRenderer(draw_params=draw_params, figsize=(25, 20), plot_limits=[400, 900, 650, 780])
    # rnd = MPRenderer(draw_params=draw_params, figsize=(25, 20))
    # sc.draw(rnd)
    # pp.draw(rnd)
    # rnd.render()
    # plt.show()
    ego_params = copy.deepcopy(draw_params)
    ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
    sc.remove_obstacle(ego)
    rnd.create_video([sc, ego], "scenario_with_ego.gif", draw_params=[draw_params, ego_params])


if __name__ == "__main__":
    import matplotlib

    matplotlib.use("TkAgg")
    main()