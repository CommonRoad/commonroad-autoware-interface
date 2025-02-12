from abc import ABC, abstractmethod
from typing import Any

class Event(ABC):
    """
    Abstract base class for a event in the state machine.
    """
    def __init__(self, machine, node: Any):
        self.node = node
        self.machine = machine

    @abstractmethod
    def run(self):
        """
        Function to be executed when event is triggered.
        """
        pass

    def callback(self):
        """
        Callback function for the event.
        """
        self.machine.process_event(self)
