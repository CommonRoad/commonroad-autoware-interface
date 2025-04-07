from ..base.configuration import StateMachineConfig, StateConfig, EventConfig, TransitionConfig, SuperstateConfig
from ..base.transition import Transition
from .states.basic_states import Initialization, InteractivePlanning,InteractiveDriving, InteractiveWaiting, FollowTrajectory
from .states.interactive_planning import UpdateScenario, UpdateGoal, UpdateInitialPose, PlanRoute
from .states.interactive_waiting import UpdateScenario_Waiting, BehaviorPlanning_Waiting, PublishTrajectory_Waiting
from .states.interactive_driving import UpdateScenario_Driving, BehaviorPlanning_Driving, PublishTrajectory_Driving, CheckGoalReached_Driving
from .events.basic_events import HasSolutionPath, NoSolutionPath, PlanningFinishedEvent, AutowareEngagedEvent, GoalReachedEvent, EngageFalseEvent, ClearRouteEvent, StopButtonEvent, ChangedInitialPoseEvent
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
    ],
    transitions=[
        TransitionConfig(cls=Transition, event=NoSolutionPath, source_state=Initialization, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=HasSolutionPath, source_state=Initialization, target_state=FollowTrajectory),
        TransitionConfig(cls=Transition, event=PlanningFinishedEvent, source_state=InteractivePlanning, target_state=InteractiveWaiting),
        TransitionConfig(cls=Transition, event=AutowareEngagedEvent, source_state=InteractiveWaiting, target_state=InteractiveDriving),
        TransitionConfig(cls=Transition, event=GoalReachedEvent, source_state=InteractiveDriving, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=EngageFalseEvent, source_state=InteractiveDriving, target_state=InteractiveWaiting),
        TransitionConfig(cls=Transition, event=StopButtonEvent, source_state=InteractiveDriving, target_state=InteractiveWaiting),
        TransitionConfig(cls=Transition, event=ClearRouteEvent, source_state=InteractivePlanning, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=ClearRouteEvent, source_state=InteractiveWaiting, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=ClearRouteEvent, source_state=InteractiveDriving, target_state=InteractivePlanning),
        TransitionConfig(cls=Transition, event=ChangedInitialPoseEvent, source_state=InteractiveWaiting, target_state=UpdateInitialPose),
    ]
)
