from abc import ABC
from abc import abstractmethod


class TrajectoryPlannerInterface(ABC):
    """Abstract class for trajectory planner interface."""

    @abstractmethod
    def plan(self):
        pass
