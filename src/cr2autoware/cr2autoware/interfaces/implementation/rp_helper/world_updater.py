import time
from typing import Optional

from crcpp import World
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario
from rclpy.impl.rcutils_logger import RcutilsLogger

class WorldUpdater:
    _world: World
    _scenario: Scenario

    _logger: Optional[RcutilsLogger]

    def __init__(self, scenario: Scenario, world: World, logger: Optional[RcutilsLogger] = None) -> None:
        self._world = world
        self._scenario = scenario
        self._logger = logger.get_child("world_updater") if logger else None

    @property
    def world(self) -> World:
        return self._world

    def scenario_updated(self) -> None:
        tic = time.perf_counter()
        non_vrus = [
            obs
            for obs in self._scenario.obstacles
            if obs.obstacle_type not in (ObstacleType.BICYCLE, ObstacleType.PEDESTRIAN, ObstacleType.PARKED_VEHICLE)
        ]
        self._world.update_obstacles(non_vrus)
        toc = time.perf_counter()
        if self._logger:
            self._logger.info(f"Updating C++ world took {(toc - tic) * 1000:.2f} ms")
            num_obs = len(self._world.obstacles)
            self._logger.info(f"Number of obstacles in C++ world: {num_obs}")
