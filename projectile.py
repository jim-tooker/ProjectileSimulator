"""
Contains the elements for the Projectile class and Projectile types
"""
from enum import Enum, auto
import math
from dataclasses import dataclass, field
from typing import Dict, List

__author__ = "Jim Tooker"


class ProjectileType(Enum):
    """
    Enum to hold Projectile types
    """
    GOLF_BALL = auto()
    PING_PONG_BALL = auto()
    TENNIS_BALL = auto()
    BASEBALL = auto()
    BOWLING_BALL = auto()
    SHOT_PUT = auto()
    CUSTOM = auto()

    def __str__(self) -> str:
        return self.name.replace('_', ' ').title()

@dataclass
class Projectile:
    """
    Dataclass to hold the Projectile information.

    Attributes:
        type (ProjectileType): The type of the projectile.
        mass (float): The mass of the projectile (kg).
        radius (float): The radius of the projectile (m).
        area (float): The area of the projectile (m^2).
        display_name (str): Formatted name of the projectile.
    """
    type: ProjectileType
    mass: float  # kg
    radius: float  # m
    area: float = field(init=False)  # m^2
    display_name: str = field(init=False)

    def __post_init__(self) -> None:
        self.display_name = f'{str(self.type):<15}'
        self.area = math.pi * self.radius**2

    @classmethod
    def create_dict(cls) -> Dict[ProjectileType, 'Projectile']:
        """
        Creates a Dict of the "canned" projectiles available.
        """
        projectiles: List[Projectile] = [
            cls(type=ProjectileType.GOLF_BALL, mass=0.046, radius=0.0213),
            cls(type=ProjectileType.PING_PONG_BALL, mass=0.0027, radius=0.02),
            cls(type=ProjectileType.TENNIS_BALL, mass=0.058, radius=0.0337),
            cls(type=ProjectileType.BASEBALL, mass=0.145, radius=0.0373),
            cls(type=ProjectileType.BOWLING_BALL, mass=6, radius=0.108),
            cls(type=ProjectileType.SHOT_PUT, mass=7.26, radius=0.0625),
        ]
        return {proj.type: proj for proj in projectiles}

PROJECTILES = Projectile.create_dict()
