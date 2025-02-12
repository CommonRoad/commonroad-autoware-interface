from cr2autoware.state_machine.base.event import Event

class UpdateScenarioEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Scenario Updated initiated.")

class UpdateGoalEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Goal Updated initiated.")

class PlanRouteEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Route Planning initiated.")
    
class UpdateInitialPoseEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Initial Pose Update initiated.")

class UpdateReferencePathEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Reference Path Publishing initiated.")
    
class PublishTrajectoryEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Trajectory Publishing initiated.")

class CheckGoalReachedEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Check Goal Reached initiated.")

class BehaviorPlanningEvent(Event):
    """
    Event for initialization of the state machine.
    """

    def __init__(self, machine, node):
        super().__init__(machine, node)

    def run(self):
        self.node._logger.debug("Run Behavior Planning.")
