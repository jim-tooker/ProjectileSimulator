"""
Contains the elements for the Environment class and Environment types
"""
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, List

__author__ = "Jim Tooker"


class EnvironmentType(Enum):
    """
    Enum to hold Environment types
    """
    EARTH = auto()
    EARTH_NO_AIR = auto()
    MARS = auto()
    MOON = auto()
    VENUS = auto()
    JUPITER = auto()
    CUSTOM = auto()

    def __str__(self) -> str:
        return self.name.replace('_', ' ').title()


@dataclass
class Environment:
    """
    Dataclass to hold the Environment information.

    Attributes:
        type (EnvironmentType): The type of the environment.
        gravity (float): The gravity of the environment (m/s^2).
        air_density (float): The air density of the environment (kg/m^3).
        display_name (str): Formatted name of the environment.
    """
    type: EnvironmentType
    gravity: float   # m/s^2
    air_density: float  # kg/m^3
    display_name: str = field(init=False)

    def __post_init__(self) -> None:
        self.display_name = f'{str(self.type):<15}'

    @classmethod
    def create_dict(cls) -> Dict[EnvironmentType, 'Environment']:
        """
        Creates a Dict of the "canned" environments available.
        """
        environments: List[Environment] = [
            cls(type=EnvironmentType.EARTH, gravity=9.81, air_density=1.225),
            cls(type=EnvironmentType.EARTH_NO_AIR, gravity=9.81, air_density=0),
            cls(type=EnvironmentType.MARS, gravity=3.72, air_density=0.02),
            cls(type=EnvironmentType.MOON, gravity=1.62, air_density=0),
            cls(type=EnvironmentType.VENUS, gravity=8.87, air_density=65),
            cls(type=EnvironmentType.JUPITER, gravity=24.79, air_density=0.16),
        ]
        return {env.type: env for env in environments}

ENVIRONMENTS = Environment.create_dict()
