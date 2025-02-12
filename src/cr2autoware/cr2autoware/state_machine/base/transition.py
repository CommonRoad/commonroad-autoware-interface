from abc import ABC


class Transition(ABC):
    """
    Abstract base class for a transition in the state machine.
    """

    def __init__(self, machine, event, source_state, target_state):
        self.node = machine.node
        self.event = event
        self.source_state = source_state
        self.target_state = target_state

    def is_triggered(self, current_state, event):
        """
        Checks for a given event if the transition is triggered.
        """
        return isinstance(event, self.event) and isinstance(current_state, self.source_state)
