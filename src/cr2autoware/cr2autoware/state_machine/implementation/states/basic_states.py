from cr2autoware.state_machine.base.state import State, Superstate
from cr2autoware.state_machine.base.configuration import StateConfig

class Initialization(State):
    """
    State for initialization of the state machine.
    """

    def __init__(self, machine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        self.node.init_planner_interface()

    def _exit(self):
        """
        Exit action of the state.
        """


    def _throughout(self):
        """
        Throughout action of the state.
        """
        pass


class InteractivePlanning(Superstate):
    """
    State for planning.
    """

    def __init__(self, machine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        pass

    def _exit(self):
        """
        Exit action of the state.
        """
        pass

    def _throughout(self):
        """
        Throughout action of the state.
        """
        pass


class InteractiveWaiting(Superstate):
    """
    State for waiting.
    """

    def __init__(self, machine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        pass

    def _exit(self):
        """
        Exit action of the state.
        """
        pass

    def _throughout(self):
        """
        Throughout action of the state.
        """
        pass


class InteractiveDriving(Superstate):
    """
    State for driving.
    """

    def __init__(self, machine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        pass

    def _exit(self):
        """
        Exit action of the state.
        """
        pass

    def _throughout(self):
        """
        Throughout action of the state.
        """
        pass


class InteractiveFailSafe(Superstate):
    """
    State for fail-safe.
    """

    def __init__(self, machine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        pass

    def _exit(self):
        """
        Exit action of the state.
        """
        pass

    def _throughout(self):
        """
        Throughout action of the state.
        """
        pass


class FollowTrajectory(State):
    """
    State for following a trajectory.
    """

    def __init__(self, machine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        self.node.follow_solution_trajectory()

    def _exit(self):
        """
        Exit action of the state.
        """
        pass

    def _throughout(self):
        """
        Throughout action of the state.
        """
        self.node.follow_trajectory_mode_update()
