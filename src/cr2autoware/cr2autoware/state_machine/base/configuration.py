from dataclasses import dataclass, field
from typing import List, Any

@dataclass
class StateConfig:
    cls: Any

@dataclass
class EventConfig:
    cls: Any

@dataclass
class TransitionConfig:
    cls: Any
    event: Any
    source_state: Any
    target_state: Any

@dataclass
class StateMachineConfig:
    initial_state: Any
    states: List[StateConfig]
    events: List[EventConfig]
    transitions: List[TransitionConfig]
    history: bool = False

@dataclass
class SuperstateConfig(StateConfig):
    cls: Any
    initial_state: StateConfig
    states: List[StateConfig] = field(default_factory=list)
    events: List[EventConfig] = field(default_factory=list)
    transitions: List[TransitionConfig] = field(default_factory=list)
    history: bool = False
