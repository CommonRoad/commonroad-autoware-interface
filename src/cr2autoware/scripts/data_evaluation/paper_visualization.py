import copy
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path
from typing import List, Optional

import imageio.v3 as iio
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleType
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt

from scripts.data_evaluation.visualization import get_ego_params


@dataclass(frozen=True)
class Plot:
    base_path: Path
    name: str
    time_step: int = 0
    ego_id: int = 42
    horizon: int = 25
    plot_limits: Optional[List[float]] = None
    frame_offset: int = 0
    dt: float = 0.1
    fps: int = 30

    @cached_property
    def scenario_path(self) -> Path:
        return (
            self.base_path / "CommonRoad" / f"scenario_obstacles_ego_interpolated.xml"
        )

    @cached_property
    def save_path(self) -> Path:
        return self.base_path / "Plots" / self.name

    @cached_property
    def video_path(self) -> Path:
        return self.base_path / "edgar.mov"

    def get_frame_number(self, initial_step: int) -> int:
        frames_per_step = self.fps * self.dt
        return (
            round((self.time_step - initial_step) * frames_per_step) + self.frame_offset
        )


def main() -> None:
    base_path = Path("/home/lercher/tum/edgar/data/artifact")
    plots = [
        Plot(
            base_path / "2025-04-11_second_test_safe_dist",
            "lane_following_straight",
            time_step=850,
            plot_limits=[748, 802, 702, 723],
            frame_offset=6,
        ),
        Plot(
            base_path / "2025-04-16_first_test_safe_dist",
            "lane_following_left_corner",
            time_step=860,
            plot_limits=[870, 928, 30, 55],
            frame_offset=73,
        ),
        Plot(
            base_path / "2025-04-16_right_turn",
            "right_turn",
            time_step=650,
            plot_limits=[565, 616, 715.5, 746],
            frame_offset=-44,
        ),
    ]
    figsize = (50, 40)

    for plot in plots:
        if not plot.save_path.exists():
            plot.save_path.mkdir(parents=True)

        # Load commonroad scenario
        scenario, _ = CommonRoadFileReader(filename_2020a=plot.scenario_path).open()
        ego = scenario.obstacle_by_id(plot.ego_id)
        scenario.remove_obstacle(ego)
        scenario.remove_obstacle(
            [
                obs
                for obs in scenario.obstacles
                if obs.state_at_time(plot.time_step) is None
            ]
        )

        relevant_traffic_signs = [
            ts
            for ts in scenario.lanelet_network.traffic_signs
            if plot.plot_limits[0] <= ts.position[0] <= plot.plot_limits[1]
            and plot.plot_limits[2] <= ts.position[1] <= plot.plot_limits[3]
        ]

        def type_to_min_vel(obs_type: ObstacleType):
            match obs_type:
                case ObstacleType.CAR | ObstacleType.TRUCK | ObstacleType.BUS:
                    return 2
                case ObstacleType.BICYCLE:
                    return 2
                case ObstacleType.PEDESTRIAN:
                    return 1
                case _:
                    return 2

        standing_obstacles = [
            obs
            for obs in scenario.dynamic_obstacles
            if (state := obs.state_at_time(plot.time_step)) is not None
            and abs(state.velocity) < type_to_min_vel(obs.obstacle_type)
        ]
        scenario.remove_obstacle(standing_obstacles)

        # Draw the scenario and ego vehicle
        draw_params = MPDrawParams(
            time_begin=plot.time_step, time_end=plot.time_step + plot.horizon
        )
        draw_params.lanelet_network.traffic_light.draw_traffic_lights = False
        draw_params.lanelet_network.traffic_sign.draw_traffic_signs = False
        draw_params.dynamic_obstacle.draw_icon = True
        draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
        draw_params.dynamic_obstacle.occupancy.draw_occupancies = True
        rnd = MPRenderer(
            draw_params=draw_params,
            plot_limits=plot.plot_limits,
            figsize=figsize,
        )
        scenario.draw(rnd)
        standing_params = copy.deepcopy(draw_params)
        standing_params.time_end = plot.time_step
        for obs in standing_obstacles:
            obs.draw(rnd, draw_params=standing_params)
        ego_params = get_ego_params(draw_params)
        ego.draw(rnd, draw_params=ego_params)
        for ts in relevant_traffic_signs:
            ts.draw(rnd)

        # Plot settings
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        plt.margins(0, 0)

        # Render the plot
        rnd.render()

        # Save the plot
        plt.axis("off")
        # plt.show()
        plt.savefig(
            plot.save_path / f"commonroad.svg",
            format="svg",
            bbox_inches="tight",
            pad_inches=0,
            transparent=True,
        )

        # Save the corresponding video frame
        frame = iio.imread(
            plot.video_path,
            index=plot.get_frame_number(ego.initial_state.time_step),
            plugin="pyav",
        )
        iio.imwrite(plot.save_path / f"edgar.png", frame)


if __name__ == "__main__":
    import matplotlib

    matplotlib.use("TkAgg")
    main()
