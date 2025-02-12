from cr2autoware.state_machine.base.state import State
from cr2autoware.state_machine.base.state_machine import StateMachine
from cr2autoware.state_machine.base.configuration import StateConfig
from cr2autoware.state_machine.implementation.events.interactive_planning import UpdateScenarioEvent, UpdateGoalEvent, UpdateInitialPoseEvent, PlanRouteEvent
from cr2autoware.state_machine.implementation.events.basic_events import PlanningFinishedEvent

class UpdateScenario(State):
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

        if self.node.new_initial_pose:
            self.machine.process_event(UpdateInitialPoseEvent(self.machine, self.node))
        elif not self.node.route_planner.is_route_planned:
            self.machine.process_event(UpdateGoalEvent(self.machine, self.node))
        else:
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
        self.node.update_scenario()


        if self.node.new_initial_pose:
            self.machine.process_event(UpdateInitialPoseEvent(self.machine, self.node))
        elif not self.node.route_planner.is_route_planned:
            self.machine.process_event(UpdateGoalEvent(self.machine, self.node))
        else:
            self.machine.process_event(UpdateScenarioEvent(self.machine, self.node))


class UpdateInitialPose(State):
    """
    Update Initial Pose.
    """

    def __init__(self, machine: StateMachine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        self.node.update_initial_pose()

        if not self.node.route_planner.is_route_planned:
            self.machine.process_event(UpdateGoalEvent(self.machine, self.node))
        else: 
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

class UpdateGoal(State):
    """
    State for Goal Update.
    """

    def __init__(self, machine: StateMachine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        if len(self.node.goal_msgs) > 0:
            goal_set = self.node.update_goal()
            if goal_set:
                self.machine.process_event(PlanRouteEvent(self.machine, self.node))
            else:
                self.machine.process_event(UpdateScenarioEvent(self.machine, self.node))
        else:
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

class PlanRoute(State):
    """
    State for Route and Velocity Planning.
    """

    def __init__(self, machine: StateMachine, config: StateConfig):
        super().__init__(machine, config)

    def _entry(self):
        """
        Entry action of the state.
        """
        self.node.plan_route()

        self.machine.process_event(PlanningFinishedEvent(self.machine, self.node))

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
