from cr2autoware.state_machine.base.event import Event

class HasSolutionPath(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[state_machine] Initialization finished. Solution path found.")

class NoSolutionPath(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[state_machine] Initialization finished. No solution path found.")

class PlanningFinishedEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[state_machine] Planning finished.")
    
class AutowareEngagedEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[state_machine] Autoware engaged.")

class GoalReachedEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[state_machine] Goal reached.")

class EngageFalseEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[state_machine] Engage False.")

class ClearRouteEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node.clear_route()

        self.node._logger.debug("[state_machine] Clear Route.")

class StopButtonEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node.plan_route()

        self.node._logger.debug("[state_machine] Stop Button pressed.")

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

        self.node._logger.debug("[state_machine] Initial Pose Update initiated.")

class SlowdownEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("[state_machine] Slowdown triggered.")
