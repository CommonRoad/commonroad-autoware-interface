from cr2autoware.state_machine.base.state import State
from cr2autoware.state_machine.base.state_machine import StateMachine
from cr2autoware.state_machine.base.configuration import StateConfig
from cr2autoware.state_machine.implementation.events.interactive_planning import BehaviorPlanningEvent, PublishTrajectoryEvent, UpdateScenarioEvent

class UpdateScenario_Slowdown(State):
    """
    State for Scenario Update.
    """

    def __init__(self, machine: StateMachine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        self.node.update_scenario()

        if self.node.route_planner.is_route_planned:
            self.machine.process_event(BehaviorPlanningEvent(self.machine, self.node))
        else:
            raise Exception("Route is not planned yet.")

    def _exit(self):
        """
        Exit action of the state.
        """
        pass

    def _throughout(self):
        """
        Throughout action of the state.
        """
        self.node.update_scenario()

        if self.node.route_planner.is_route_planned:
            self.machine.process_event(BehaviorPlanningEvent(self.machine, self.node))
        else:
            raise Exception("Route is not planned yet.")


class BehaviorPlanning_Slowdown(State):
    """
    State for Behavior Planning.
    """
    
    def __init__(self, machine: StateMachine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        self.node.behavior_slowdown()

        self.machine.process_event(PublishTrajectoryEvent(self.machine, self.node))

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


class PublishTrajectory_Slowdown(State):
    """
    State for Route Planning.
    """

    def __init__(self, machine: StateMachine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        self.node.publish_trajectory()

        self.machine.process_event(UpdateScenarioEvent(self.machine, self.node))

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
