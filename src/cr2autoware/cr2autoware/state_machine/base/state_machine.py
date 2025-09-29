from .transition import Transition
from .state import State, Superstate
from .configuration import StateMachineConfig, SuperstateConfig
from .event import Event
import numpy as np
import queue
import threading
from typing import List, Any

class StateMachine:
    """
    # State Machine Package

    In CommonRoad2Autoware, a custom state machine is implemented to handle transitions between different states based on events.

    - **Base State Machine**: Handles the initialization and structure of the state machine. It manages the processing of events and coordinates the execution of transitions and state actions.

    - **States**: Provides the base class for states in the state machine. States define actions for entry, exit, and throughout phases. Additionally, superstates can be defined, which group multiple states as children to represent hierarchical state structures.

    - **Events**: Contains the base class for events that trigger transitions between states. Events are triggered by the node and encapsulate the logic for when and how state transitions occur.

    - **Transitions**: Defines the logic for transitioning from a source state to a target state.

    - **Configuration**: Defines the configuration classes for states, events, transitions, and the state machine itself. These classes are used to set up the structure and behavior of the state machine.
    
    ---

    ## Minimal Example

    Below is a minimal Python example that demonstrates the creation of two states, an event, and a transition between the states using the `state_machine` package:

    ```python
    from cr2autoware.cr2autoware.state_machine.base.state_machine import StateMachine
    from cr2autoware.cr2autoware.state_machine.base.state import State
    from cr2autoware.cr2autoware.state_machine.base.event import Event
    from cr2autoware.cr2autoware.state_machine.base.transition import Transition
    from cr2autoware.cr2autoware.state_machine.base.configuration import (
        StateConfig,
        EventConfig,
        TransitionConfig,
        StateMachineConfig,
    )

    # Define two simple states
    class StateOne(State):
        def _entry(self):
            print("Entry State One")
        
        def _exit(self):
            print("Entry State One")

        def _throughout(self):
            print("Throughout State One")

    class StateTwo(State):
        def _entry(self):
            print("Entering State Two")
        
        def _exit(self):
            print("Exiting State Two")

        def _throughout(self):
            print("Throughout State Two")

    # Define an event
    class EventOne(Event):
        def run(self):
            print("Event triggered: EventOne")


    # Create the state machine configuration
    state_one_config = StateConfig(cls=StateOne)
    state_two_config = StateConfig(cls=StateTwo)
    event_one_config = EventConfig(cls=EventOne)

    transition_config = TransitionConfig(
        cls=Transition,  
        event=event_one_config,
        source_state=state_one_config,
        target_state=state_two_config,
    )

    state_machine_config = StateMachineConfig(
        initial_state=state_one_config,
        states=[state_one_config, state_two_config],
        events=[event_one_config],
        transitions=[transition_config],
    )

    # Placeholder Node:
    class StateMachineNode(rclpy.Node):
        pass

    # Init State Machine:
    state_machine = StateMachine(StateMachineNode, state_machine_config)

    # Trigger the event to transition to StateTwo:
    state_machine.process_event(EventOne(state_machine, StateMachineNode))
    ```
    """
    def __init__(self, node: Any, config: StateMachineConfig):
        self.lock = threading.RLock()
        self.node = node
        self.history: bool = False
        self.initial_state = None
        self.states = np.array([])
        self.transitions = np.array([])
        self.events = np.array([])
        # TODO: ADD Priority Queue
        self.event_queue = queue.Queue()
        self.current_state = None
        self.stopping = False
        self.running = False
        self.history_list = np.array([])
        self.load_config(config)
        self.init_states()

    def load_config(self, config: StateMachineConfig):
        """
        Load the configuration of the state machine.

        :param config: configuration of the state machine
        """
        with self.lock:
            self.initial_state: State = config.initial_state.cls(self, config.initial_state)
            self.add_state(self.initial_state)

            self.history = config.history

            # Load states
            for state in config.states:
                self.add_state(state.cls(self, state))

            # Load events
            for event in config.events:
                self.add_event(event.cls(self, self.node))

            # Load transitions
            for transition in config.transitions:
                self.add_transition(transition.cls(self, transition.event, transition.source_state, transition.target_state))

    def init_states(self):
        """
        Initialize the states. Add the corresponding superstate to each substate.
        """
        def init_substates(superstate: Superstate):
            for state in superstate.substates:
                if isinstance(state, Superstate):
                    init_substates(state)
                state.superstate = superstate

        for state in self.states:
            if isinstance(state, Superstate):
                init_substates(state)
            

    def add_state(self, state: State):
        """
        Add a state to the state machine.

        :param state: state to be added
        """
        self.states = np.append(self.states, state)
    
    def add_transition(self, transition: Transition):
        """
        Add a transition to the state machine.

        :param transition: transition to be added
        """
        self.transitions = np.append(self.transitions, transition)

    def add_event(self, event: Event):
        """
        Add an event to the state machine.

        :param event: event to be added
        """
        self.events = np.append(self.events, event)

    def start(self):
        """
        Start the state machine.
        """
        with self.lock:
            if self.current_state is not None:
                raise Warning("State machine is already running")
            else:
                self.node._logger.debug("[SVEN]Start state machine")
                self.current_state = self.initial_state
                self.running = True
                self.current_state.activate()
                self._check_event_queue()
                self.running = False

            self._locked = False

    def stop(self):
        """
        Stop the state machine.
        """
        with self.lock:
            if self.current_state is None:
                raise Warning("State machine is not running")
            else:
                self.stopping = True
                self.node._logger.debug("[SVEN]Stop state machine")
                self.running = True
                self.current_state.deactivate()
                self.current_state = None
                self._check_event_queue()
                self.running = False
                self.node.destroy_node()
                self.stopping = False
    
    def process_event(self, event: Event):
        """
        Process an event.

        :param event: event to be processed
        """
        # TODO: ADD Priority Queue
        self.event_queue.put(event)

        if not self.running:
            self.running = True
            self._check_event_queue()
            self.running = False

    def _check_event_queue(self):
        """
        Check the event queue for new events.
        """
        while not self.event_queue.empty():
            event = self.event_queue.get()
            self._handle_event(event)
            self.event_queue.task_done()

        if not self.stopping:
            raise Exception("No Event found in the queue!")

    def _handle_event(self, event: Event):
        """
        Handle an event.
        """
        with self.lock:
            # None is returned if no transition is found
            new_state = self.current_state.handle_event(event)

            # if new_state is a class, get the state instance from the configuration
            # if new_state is a class, new_state is the target state of the transition and not found in self.current_state
            # therfore search in self.states or in substates of superstates
            if not isinstance(new_state, State):
                new_state = self._get_state_from_config(new_state)
            
            # if new_state is None, the current state is throughout
            if new_state is None:
                self.current_state.throughout()
            else:
                # update current state and activate it
                self.current_state = new_state
                self.current_state.activate()
                self.history = np.append(self.history, self.current_state)

    def _get_state_from_config(self, state_cls: State) -> State:
        """
        Get the state instance from the configuration via the state class.

        :param state_cls: state class
        :return: state
        """
        if state_cls is None:
            return None
        
        if not isinstance(state_cls, type):
            raise ValueError(f"{state_cls.__class__} is missing in a SuperstateConfig!")

        # return the state instance from the configuration if state_cls is in self.states
        for state in self.states:
            if isinstance(state, state_cls):
                return state
            
        # if state_cls is not found in self.states, search in superstates
        for state in self.states:
            # skip the current state, since it was already checked in handle_event
            if state == self.current_state:
                continue

            if isinstance(state, Superstate):
                new_state = state.search_in_substate(state_cls)
                if isinstance(new_state, State):
                    return new_state
            
        raise ValueError(f"State {state_cls} not found in configuration!")
