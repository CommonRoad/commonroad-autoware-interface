# standard imports
from typing import Any
from dataclasses import dataclass, field


@dataclass
class BaseSpec:
    """Base class for ROS interface specification"""

    # name (i.e., ROS topic) of the interface
    name: str

    def __getitem__(self, item: str) -> Any:
        """Getter for base parameter value."""
        try:
            value = self.__getattribute__(item)
        except AttributeError as e:
            raise KeyError(f"{item} is not a parameter of {self.__class__.__name__}") from e
        return value

    def __setitem__(self, key: str, value: Any):
        """Setter for item."""
        try:
            self.__setattr__(key, value)
        except AttributeError as e:
            raise KeyError(f"{key} is not a parameter of {self.__class__.__name__}") from e


@dataclass
class PublisherSpec(BaseSpec):
    """Class for topic publisher specification"""
    pass


@dataclass
class SubscriptionSpec(BaseSpec):
    """Class for topic subscription specification"""
    pass


@dataclass
class SrvClientSpec(BaseSpec):
    """Class for service client specification"""
    pass
