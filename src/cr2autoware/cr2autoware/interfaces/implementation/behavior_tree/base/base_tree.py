from abc import ABC, abstractmethod
from rclpy.impl.rcutils_logger import RcutilsLogger
from py_trees.behaviour import Behaviour
from typing import Type

class BaseTree(ABC):
    """
    Base class for behavior trees.

    :var logger: ROS2 node logger
    :var verbose: Flag for verbose logging
    :var root: Root node of the behavior tree
    """
    def __init__(self, logger: RcutilsLogger, verbose: bool = False):
        self.logger = logger
        self.verbose = verbose
        self.root = None

    @abstractmethod
    def create_behavior_tree(self) -> Type[Behaviour]:
        """
        Create a behavior tree and return its root node.

        :return: A subclass of Behaviour representing the root node of the behavior tree.
        """
        pass

    def tick_once(self):
        """
        Execute one tick of the behavior tree.
        """
        self.root.tick_once()
