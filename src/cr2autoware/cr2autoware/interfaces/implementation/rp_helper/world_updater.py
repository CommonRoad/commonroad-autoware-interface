import time
from typing import List, Optional

from crcpp import World
from commonroad.scenario.obstacle import Obstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from rclpy.impl.rcutils_logger import RcutilsLogger

class WorldUpdater:
    _world: World
    _scenario: Scenario

    _logger: Optional[RcutilsLogger]

    def __init__(self, scenario: Scenario, world: Optional[World] = None, logger: Optional[RcutilsLogger] = None) -> None:
        self._scenario = scenario
        self._world = world if world is not None else self._init_world(scenario)
        self._logger = logger.get_child("world_updater") if logger else None

    @property
    def world(self) -> World:
        return self._world

    def scenario_updated(self) -> None:
        tic = time.perf_counter()
        filtered_obstacles = self._filter_obstacles(self._scenario.obstacles)
        self._world.update_obstacles(filtered_obstacles)
        toc = time.perf_counter()
        if self._logger:
            self._logger.info(f"Updating C++ world took {(toc - tic) * 1000:.2f} ms")

    @staticmethod
    def _filter_obstacles(obstacles: List[Obstacle]) -> List[Obstacle]:
        return [
            obs
            for obs in obstacles
            if obs.obstacle_type not in (ObstacleType.BICYCLE, ObstacleType.PEDESTRIAN, ObstacleType.PARKED_VEHICLE, ObstacleType.UNKNOWN)
        ]

    @staticmethod
    def _init_world(scenario: Scenario) -> World:
        return World(
            str(scenario.scenario_id),
            0,
            scenario.dt,
            scenario.scenario_id.country_id,
            scenario.lanelet_network,
            [],  # no ego vehicles in world so far
            WorldUpdater._filter_obstacles(scenario.obstacles),
        )
