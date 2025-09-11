from cr2autoware.state_machine.base.event import Event

class HasSolutionPath(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[SVEN]Initialization finished. Solution path found.")

class NoSolutionPath(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[SVEN]Initialization finished. No solution path found.")

class PlanningFinishedEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[SVEN]Planning finished.")
    
class AutowareEngagedEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[SVEN]Autoware engaged.")

class GoalReachedEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[SVEN]Goal reached.")

class EngageFalseEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[SVEN]Engage False.")

class ClearRouteEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node.clear_route()

        self.node._logger.debug("[SVEN]Clear Route.")

class StopButtonEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node.plan_route()

        self.node._logger.debug("[SVEN]Stop Button pressed.")

class ChangedInitialPoseEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        # Reset the goal messages and clear the route
        self.node.goal_msgs = []
        self.node.clear_route()
        self.node.send_clear_route_srv_request()

        self.node._logger.debug("[SVEN]Initial Pose Update initiated.")

class SlowdownEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[SVEN]Slowdown triggered.")
