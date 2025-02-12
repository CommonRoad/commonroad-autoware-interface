from abc import ABC, abstractmethod
import numpy as np
from typing import Type
from .configuration import StateConfig, SuperstateConfig

class State(ABC):
    """
    Abstract base class for a state in the state machine.
    """
    def __init__(self, machine, config: StateConfig):
        self.superstate = None
        self.machine = machine
        self.node = machine.node
        self.activated = False
        self.lock = machine.lock
    
    def activate(self):
        """
        Activate the state. Returns the corresponding superstate if the state is a substate.
        """
        with self.lock:
            if not self.activated:
                self.node._logger.debug(f"Activate state: {self.__class__.__name__}")
                self.activated = True
                # activate superstate
                if self.superstate:
                    self.superstate.activate(self)

                # activate this state
                self._entry()
                self.node._logger.debug(f"Entry finished: {self.__class__.__name__}")

            else:
                self.node._logger.debug(f"State already activated: {self.__class__.__name__}")

    def deactivate(self):
        """
        Deactivate the state.
        """
        with self.lock:
            if self.activated:
                self.activated = False
                self.node._logger.debug(f"Deactivate state: {self.__class__.__name__}")
                self._exit()
            else:
                self.node._logger.debug(f"State already deactivated: {self.__class__.__name__}")


    def throughout(self):
        """
        Execute events and outputs throughout the state.
        """
        with self.lock:
            self.node._logger.debug(f"Throughout state: {self.__class__.__name__}")
            self._throughout()

    @abstractmethod
    def _entry(self):
        """
        Events and outputs to be executed when entering the state.
        """
        pass

    @abstractmethod
    def _exit(self):
        """
        Events and outputs to be executed when exiting the state.
        """
        pass

    @abstractmethod
    def _throughout(self):
        """
        Events and outputs to be executed throughout the state.
        """
        pass

    def handle_event(self, event) -> Type['State']:
        """
        Event handler for the state. Checks if an event triggers a transition.

        Returns the corresponding state if a transition is triggered. Returns None if no transition is triggered.

        :param event: event to be handled
        """
        with self.lock:
            # TODO: check for multiple transitions / AND / OR  / parallel transitions
            if self.superstate is not None:
                transitions = self.superstate.transitions
            else:
                transitions = self.machine.transitions

            for transition in transitions:
                if transition.is_triggered(self, event):
                    self.deactivate()
                    event.run()

                    return transition.target_state
                    
            return None


class Superstate(State):
    """
    Abstract base class for a superstate in the state machine.
    """

    def __init__(self, machine, config: SuperstateConfig):
        super().__init__(machine, config)
        self.substates = np.array([])
        self.events = np.array([])
        self.transitions = np.array([])
        self.current_substate = None
        self.initial_state = None
        self.history: bool = False
        self.history_list = np.array([])
        self.load_config(config)

    def activate(self, substate: State = None):
        """
        Activate the superstate.

        :param substate: substate to be activated
        """
        with self.lock:
            if not self.activated:
                self.activated = True
                # activate superstate
                self.node._logger.debug(f"Activate superstate: {self.__class__.__name__}; Substate: {self.current_substate.__class__.__name__}")

                if self.superstate:
                    self.superstate.activate(self)

                # activate this state
                self._entry()

                # activate substate
                if substate:
                    # check if substate is a substate of the superstate
                    found_substate = False
                    for state in self.substates:
                        if isinstance(state, substate):
                            self.current_substate = substate
                            found_substate = True
                            break
                    if not found_substate:
                        raise ValueError("Substate not found")

                elif self.history:
                    self.current_substate = self.history_list[-1]
                else:
                    self.current_substate = self.initial_state
                self.history_list = np.array([self.current_substate])
                self.current_substate.activate()


            else:
                self.node._logger.debug(f"Superstate already activated: {self.__class__.__name__}")
 

    def deactivate(self):
        """
        Deactivate active substate.
        """
        with self.lock:
            if self.activated:
                self.activated = False
                self.node._logger.debug(f"Deactivate superstate: {self.__class__.__name__}; Substate: {self.current_substate.__class__.__name__}")
                self.current_substate.deactivate()
                self.current_substate = None
                self._exit()
            else:
                self.node._logger.debug(f"Superstate already deactivated: {self.__class__name__}")

    def add_state(self, state: State):
        """
        Add a state to the superstate.

        :param state: state to be added
        """
        self.substates = np.append(self.substates, state)

    def add_event(self, event):
        """
        Add an event to the superstate.

        :param event: event to be added
        """
        self.events = np.append(self.events, event)

    def add_transition(self, transition):
        """
        Add a transition to the superstate.

        :param transition: transition to be added
        """
        self.transitions = np.append(self.transitions, transition)

    def handle_event(self, event) -> Type[State]:
        """
        Event handler for the state. Checks if an event triggers a transition.

        Returns the corresponding state if a transition is triggered. Returns None if no transition is triggered.

        :param event: event to be handled
        :param event_check: Flag to check for transitions
        :param new_state: new state to be activated
        :param search_superstate: Flag to search for the highest superstate
        """
        with self.lock:
            # First check if an event triggers a transition in this state
            if self.superstate is not None:
                transitions = self.superstate.transitions
            else:
                transitions = self.machine.transitions
    
            for transition in transitions:
                if transition.is_triggered(self, event):
                    self.deactivate()
                    event.run()

                    return transition.target_state
            
            # Check if event triggers a transition in the substate
            target_state = self.current_substate.handle_event(event)

            # if target_state is not None, no transition in the superstate is triggered
            # the current state is still active, return None
            if target_state is None:
                return None

            return self.search_in_substate(target_state)

    def search_in_substate(self, target_state: State) -> Type[State]:
        """
        Search for the target state in the substates of the superstate.

        :param target_state: state to be searched for
        :return: self if the target state is a substate of the superstate, if not return the target state
        """
        # check if new state is a substate of the superstate
        for substate in self.substates:
            # skip the current substate, already checked in self.current_substate.handle_event(event)
            if substate == self.current_substate:
                continue

            # if the new state is a substate of the superstate
            if isinstance(substate, target_state):
                # deactivate current substate and return None, since the superstate is still active
                self.current_substate = substate
                self.current_substate.activate()
                return self
            
            # search for superstates with substates that contain the new state
            def search_substates(superstate: Superstate):
                for substate in superstate.substates:
                    if isinstance(substate, target_state):
                        # if the new state is a substate of the superstate
                        # set new state as current substate and activate it
                        # return the superstate
                        superstate.current_substate = substate
                        superstate.current_substate.activate()
                        return superstate
                    if isinstance(substate, Superstate):
                        # if the substate is a superstate, search for substates
                        # return the superstate if the new state is a substate of the superstate
                        new_current_substate = search_substates(substate)
                        if new_current_substate is not None:
                            superstate.current_substate = new_current_substate
                            superstate.current_substate.activate()
                            return superstate
                return None

            # if substate is a superstate, search for target state in its substates
            if isinstance(substate, Superstate):
                new_current_substate = search_substates(substate)
                if new_current_substate is not None:
                    self.current_substate = new_current_substate
                    self.current_substate.activate()
                    return self
        
        # if the new state is not a substate of the superstate, return the target state
        return target_state

    def load_config(self, config: SuperstateConfig):
        """
        Load the configuration of the state machine.

        :param config: configuration of the state machine
        """
        with self.lock:
            self.initial_state: State = config.initial_state.cls(self.machine, config.initial_state)
            self.add_state(self.initial_state)

            self.history = config.history

            # Load states
            for state in config.states:
                self.add_state(state.cls(self.machine, state))
    
            # Load events
            for event in config.events:
                self.add_event(event.cls(self.machine, self.node))

            # Load transitions
            for transition in config.transitions:
                self.add_transition(transition.cls(self, transition.event, transition.source_state, transition.target_state))
