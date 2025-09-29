"""
# Implementation State Machine CR2AW

The `implementation` folder contains the specific implementation of the state machine, including events and states for the CommonRoad2Autoware interface.

The following diagram provides an overview of the key state machine structure and the superstates:

![State Machine Overview](assets/state_machine_overview.svg)

The detailed structure of the state machine with all states, events, and transitions is illustrated below:

![State Machine Implementation Overview](assets/state_machine_structure.svg)

This diagram was generated using [Visual Paradigm](https://www.visual-paradigm.com/). The file is located at `docs/assets/state_machine_structure.vpp`.

---

## Config

In `config.py`, the structure of the state machine is defined. This includes the initial state, available (super)states, events, and transitions.

---

## Events

The `events` submodule contains the implementation of events that trigger transitions between states.

- **`basic_events.py`**: Handles transitions between the superstates.
- **`interactive_planning.py`**: Contains the events for transitions within the interactive superstates.

### Basic Events

::: src.cr2autoware.cr2autoware.state_machine.implementation.events.basic_events

### Interactive Planning Events

::: src.cr2autoware.cr2autoware.state_machine.implementation.events.interactive_planning

---

## States

The `states` submodule contains the implementation of states that define the behavior of the state machine.

- **`basic_states.py`**: Contains the initialization state and the superstates.
- **`interactive_planning.py`**: Contains all substates for planning the scenario, including updating the initial pose, updating the goal pose, and planning the route.
- **`interactive_waiting.py`**, **`interactive_driving.py`**, and **`interactive_slowdown.py`**: Define the substates for behavior and trajectory planning.

### Basic States

::: src.cr2autoware.cr2autoware.state_machine.implementation.states.basic_states

### Interactive Planning States

::: src.cr2autoware.cr2autoware.state_machine.implementation.states.interactive_planning

### Interactive Waiting States

::: src.cr2autoware.cr2autoware.state_machine.implementation.states.interactive_waiting

### Interactive Driving States

::: src.cr2autoware.cr2autoware.state_machine.implementation.states.interactive_driving

### Interactive Slowdown States

::: src.cr2autoware.cr2autoware.state_machine.implementation.states.interactive_slowdown
"""

from ..base.configuration import StateMachineConfig, StateConfig, EventConfig, TransitionConfig, SuperstateConfig
from ..base.transition import Transition
from .states.basic_states import Initialization, InteractivePlanning,InteractiveDriving, InteractiveWaiting, InteractiveSlowdown, FollowTrajectory
from .states.interactive_planning import UpdateScenario, UpdateGoal, UpdateInitialPose, PlanRoute
from .states.interactive_waiting import UpdateScenario_Waiting, BehaviorPlanning_Waiting, PublishTrajectory_Waiting
from .states.interactive_driving import UpdateScenario_Driving, BehaviorPlanning_Driving, PublishTrajectory_Driving, CheckGoalReached_Driving
from .states.interactive_slowdown import UpdateScenario_Slowdown, BehaviorPlanning_Slowdown, PublishTrajectory_Slowdown
from .events.basic_events import HasSolutionPath, NoSolutionPath, PlanningFinishedEvent, AutowareEngagedEvent, GoalReachedEvent, EngageFalseEvent, ClearRouteEvent, StopButtonEvent, ChangedInitialPoseEvent, SlowdownEvent
from .events.interactive_planning import UpdateScenarioEvent, UpdateGoalEvent, UpdateInitialPoseEvent, PublishTrajectoryEvent, CheckGoalReachedEvent, PlanRouteEvent, BehaviorPlanningEvent

config = StateMachineConfig(
    initial_state=StateConfig(cls=Initialization),
    history=False,
    states=[
        StateConfig(cls=Initialization),
        SuperstateConfig(cls=InteractivePlanning, 
                         initial_state=StateConfig(cls=UpdateScenario), 
                         states=[StateConfig(cls=UpdateScenario), StateConfig(cls=UpdateInitialPose), StateConfig(cls=UpdateGoal), StateConfig(cls=PlanRoute)], 
                         events=[EventConfig(cls=UpdateInitialPoseEvent), EventConfig(cls=UpdateGoalEvent), EventConfig(cls=UpdateScenarioEvent), EventConfig(cls=PlanRouteEvent)], 
                         transitions=[TransitionConfig(cls=Transition, event=UpdateInitialPoseEvent, source_state=UpdateScenario, target_state=UpdateInitialPose),
                                      TransitionConfig(cls=Transition, event=UpdateGoalEvent, source_state=UpdateScenario, target_state=UpdateGoal),
                                      TransitionConfig(cls=Transition, event=UpdateGoalEvent, source_state=UpdateInitialPose, target_state=UpdateGoal),
                                      TransitionConfig(cls=Transition, event=UpdateScenarioEvent, source_state=UpdateInitialPose, target_state=UpdateScenario),
                                      TransitionConfig(cls=Transition, event=UpdateScenarioEvent, source_state=UpdateGoal, target_state=UpdateScenario),
                                        TransitionConfig(cls=Transition, event=PlanRouteEvent, source_state=UpdateGoal, target_state=PlanRoute),
                                      TransitionConfig(cls=Transition, event=UpdateScenarioEvent, source_state=UpdateScenario, target_state=UpdateScenario)]),
        SuperstateConfig(cls=InteractiveWaiting, 
                         initial_state=StateConfig(cls=UpdateScenario_Waiting), 
                         states=[StateConfig(cls=UpdateScenario_Waiting), StateConfig(cls=BehaviorPlanning_Waiting), StateConfig(cls=PublishTrajectory_Waiting)], 
                         events=[EventConfig(cls=UpdateScenarioEvent), EventConfig(cls=BehaviorPlanningEvent), EventConfig(cls=PublishTrajectoryEvent)], 
                         transitions=[TransitionConfig(cls=Transition, event=BehaviorPlanningEvent, source_state=UpdateScenario_Waiting, target_state=BehaviorPlanning_Waiting),
                                      TransitionConfig(cls=Transition, event=PublishTrajectoryEvent, source_state=BehaviorPlanning_Waiting, target_state=PublishTrajectory_Waiting),
                                      TransitionConfig(cls=Transition, event=UpdateScenarioEvent, source_state=PublishTrajectory_Waiting, target_state=UpdateScenario_Waiting),
                                      ]
                                        ),
        SuperstateConfig(cls=InteractiveDriving, 
                         initial_state=StateConfig(cls=UpdateScenario_Driving), 
                         states=[StateConfig(cls=UpdateScenario_Driving), StateConfig(cls=BehaviorPlanning_Driving), StateConfig(cls=PublishTrajectory_Driving), StateConfig(cls=CheckGoalReached_Driving)], 
                         events=[EventConfig(cls=UpdateScenarioEvent), EventConfig(cls=BehaviorPlanningEvent), EventConfig(cls=PublishTrajectoryEvent), EventConfig(cls=CheckGoalReachedEvent)], 
                         transitions=[TransitionConfig(cls=Transition, event=UpdateScenarioEvent, source_state=CheckGoalReached_Driving, target_state=UpdateScenario_Driving),
                                      TransitionConfig(cls=Transition, event=BehaviorPlanningEvent, source_state=UpdateScenario_Driving, target_state=BehaviorPlanning_Driving),
                                      TransitionConfig(cls=Transition, event=PublishTrajectoryEvent, source_state=BehaviorPlanning_Driving, target_state=PublishTrajectory_Driving),
                                      TransitionConfig(cls=Transition, event=CheckGoalReachedEvent, source_state=PublishTrajectory_Driving, target_state=CheckGoalReached_Driving),
                                      ]
                                        ),
        SuperstateConfig(cls=InteractiveSlowdown, 
                         initial_state=StateConfig(cls=UpdateScenario_Slowdown), 
                         states=[StateConfig(cls=UpdateScenario_Slowdown), StateConfig(cls=BehaviorPlanning_Slowdown), StateConfig(cls=PublishTrajectory_Slowdown)], 
                         events=[EventConfig(cls=UpdateScenarioEvent), EventConfig(cls=BehaviorPlanningEvent), EventConfig(cls=PublishTrajectoryEvent)], 
                         transitions=[TransitionConfig(cls=Transition, event=BehaviorPlanningEvent, source_state=UpdateScenario_Slowdown, target_state=BehaviorPlanning_Slowdown),
                                      TransitionConfig(cls=Transition, event=PublishTrajectoryEvent, source_state=BehaviorPlanning_Slowdown, target_state=PublishTrajectory_Slowdown),
                                      TransitionConfig(cls=Transition, event=UpdateScenarioEvent, source_state=PublishTrajectory_Slowdown, target_state=UpdateScenario_Slowdown),
                                      ]
                                        ),
        StateConfig(cls=FollowTrajectory)
    ],
    events=[
        EventConfig(cls=NoSolutionPath),
        EventConfig(cls=HasSolutionPath),
        EventConfig(cls=PlanningFinishedEvent),
        EventConfig(cls=AutowareEngagedEvent),
        EventConfig(cls=GoalReachedEvent),
        EventConfig(cls=EngageFalseEvent),
        EventConfig(cls=StopButtonEvent),
        EventConfig(cls=ClearRouteEvent),
        EventConfig(cls=ChangedInitialPoseEvent),
        EventConfig(cls=SlowdownEvent),
    ],
    transitions=[
        TransitionConfig(cls=Transition, event=NoSolutionPath, source_state=Initialization, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=HasSolutionPath, source_state=Initialization, target_state=FollowTrajectory),
        TransitionConfig(cls=Transition, event=PlanningFinishedEvent, source_state=InteractivePlanning, target_state=InteractiveWaiting),
        TransitionConfig(cls=Transition, event=AutowareEngagedEvent, source_state=InteractiveWaiting, target_state=InteractiveDriving),
        TransitionConfig(cls=Transition, event=GoalReachedEvent, source_state=InteractiveDriving, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=EngageFalseEvent, source_state=InteractiveDriving, target_state=InteractiveWaiting),
        TransitionConfig(cls=Transition, event=StopButtonEvent, source_state=InteractiveDriving, target_state=InteractiveWaiting),
        TransitionConfig(cls=Transition, event=SlowdownEvent, source_state=InteractiveWaiting, target_state=InteractiveSlowdown),
        TransitionConfig(cls=Transition, event=SlowdownEvent, source_state=InteractiveDriving, target_state=InteractiveSlowdown),
        TransitionConfig(cls=Transition, event=ClearRouteEvent, source_state=InteractivePlanning, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=ClearRouteEvent, source_state=InteractiveWaiting, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=ClearRouteEvent, source_state=InteractiveDriving, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=ClearRouteEvent, source_state=InteractiveSlowdown, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=ChangedInitialPoseEvent, source_state=InteractiveWaiting, target_state=UpdateInitialPose),
    ]
)
