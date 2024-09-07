"""
Test harness for ProjectileSimulator
"""
import sys
from dataclasses import dataclass
from typing import Union
import pytest

sys.path.append('.')
from projectile_sim import  ProjectileSimulator
from environment import Environment, EnvironmentType, ENVIRONMENTS
from projectile import Projectile, ProjectileType, PROJECTILES

__author__ = 'Jim Tooker'


@dataclass
class ExpectedResults:
    """
    Data class to hold the expected results of the test.
    """
    max_possible_dist: float
    max_possible_height: float
    max_possible_flight_time: float
    max_height: float
    time_to_max_height: float
    dist_at_max_height: float
    total_flight_time: float
    total_distance: float


@dataclass
class CannedInputs:
    """
    Data class to hold the Canned Inputs.
    """
    env: EnvironmentType
    proj: ProjectileType
    speed: float
    angle: float


@dataclass
class CustomInputs:
    """
    Data class to hold Custom Inputs.
    """
    env: Environment
    proj: Projectile
    speed: float
    angle: float


class TestProjectileSimulator:
    """
    Class for `pytest` testing of Projectile Simulator.
    """
    ProjectileSimulator.disable_gui(True)
    tolerance = 0.001

    @staticmethod
    def generate_test_id(inputs: Union[CannedInputs, CustomInputs]) -> str:
        """
        Generates test ids for each test case.
        """
        if isinstance(inputs, CannedInputs):
            env = ENVIRONMENTS[inputs.env]
            proj = PROJECTILES[inputs.proj]
        elif isinstance(inputs, CustomInputs):
            env = inputs.env
            proj = inputs.proj
        else:
            return str(inputs)  # Fallback for unexpected input types

        return (f"ENV:{env.type.name}(g={env.gravity:.2f},ad={env.air_density:.2f}) "
               f"PROJ:{proj.type.name}(m={proj.mass:.2f},r={proj.radius:.2f}) "
               f"IN:(s={inputs.speed:.1f},a={inputs.angle:.1f})")


    def check_results(self, sim: ProjectileSimulator, expected: ExpectedResults) -> None:
        """
        Checks results of ProjectileSimulator run.
        """
        assert sim.max_possible_dist == pytest.approx(expected.max_possible_dist, abs=self.tolerance)
        assert sim.max_possible_height == pytest.approx(expected.max_possible_height, abs=self.tolerance)
        assert sim.max_possible_flight_time == pytest.approx(expected.max_possible_flight_time, abs=self.tolerance)
        assert sim.max_height == pytest.approx(expected.max_height, abs=self.tolerance)
        assert sim.time_to_max_height == pytest.approx(expected.time_to_max_height, abs=self.tolerance)
        assert sim.dist_at_max_height == pytest.approx(expected.dist_at_max_height, abs=self.tolerance)
        assert sim.total_flight_time == pytest.approx(expected.total_flight_time, abs=self.tolerance)
        assert sim.total_distance == pytest.approx(expected.total_distance, abs=self.tolerance)


    @pytest.mark.parametrize('inputs, expected', [
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=20, angle=45),
         ExpectedResults(max_possible_dist=40.775,
                         max_possible_height=20.387,
                         max_possible_flight_time=4.077,
                         max_height=10.194,
                         time_to_max_height=1.443,
                         dist_at_max_height=20.413,
                         total_flight_time=2.883,
                         total_distance=40.769)
        ),
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=20, angle=1),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=0.006,
                        time_to_max_height=0.037,
                        dist_at_max_height=0.734,
                        total_flight_time=0.069,
                        total_distance=1.386)
        ),
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=20, angle=5),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=0.155,
                        time_to_max_height=0.179,
                        dist_at_max_height=3.575,
                        total_flight_time=0.355,
                        total_distance=7.068)
        ),
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=20, angle=90),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=20.387,
                        time_to_max_height=2.039,
                        dist_at_max_height=0.0,
                        total_flight_time=4.077,
                        total_distance=0.0)
        ),
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=20, angle=88),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=20.363,
                        time_to_max_height=2.039,
                        dist_at_max_height=1.423,
                        total_flight_time=4.073,
                        total_distance=2.843)
        ),
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=1, angle=45),
        ExpectedResults(max_possible_dist=0.102,
                        max_possible_height=0.051,
                        max_possible_flight_time=0.204,
                        max_height=0.026,
                        time_to_max_height=0.073,
                        dist_at_max_height=0.051,
                        total_flight_time=0.143,
                        total_distance=0.101)
        ),
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=5, angle=45),
        ExpectedResults(max_possible_dist=2.548,
                        max_possible_height=1.274,
                        max_possible_flight_time=1.019,
                        max_height=0.637,
                        time_to_max_height=0.360,
                        dist_at_max_height=1.275,
                        total_flight_time=0.719,
                        total_distance=2.542)
        ),
        (CannedInputs(EnvironmentType.EARTH_NO_AIR, ProjectileType.GOLF_BALL, speed=100, angle=45),
        ExpectedResults(max_possible_dist=1019.368,
                        max_possible_height=509.684,
                        max_possible_flight_time=20.387,
                        max_height=254.842,
                        time_to_max_height=7.217,
                        dist_at_max_height=510.328,
                        total_flight_time=14.414,
                        total_distance=1019.214)
        ),
        (CannedInputs(EnvironmentType.EARTH, ProjectileType.GOLF_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=8.907,
                        time_to_max_height=1.309,
                        dist_at_max_height=16.849,
                        total_flight_time=2.691,
                        total_distance=32.070)
        ),
        (CannedInputs(EnvironmentType.EARTH, ProjectileType.PING_PONG_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=3.948,
                        time_to_max_height=0.754,
                        dist_at_max_height=6.044,
                        total_flight_time=1.766,
                        total_distance=9.826)
        ),
        (CannedInputs(EnvironmentType.EARTH, ProjectileType.TENNIS_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=8.001,
                        time_to_max_height=1.215,
                        dist_at_max_height=14.578,
                        total_flight_time=2.544,
                        total_distance=26.848)
        ),
        (CannedInputs(EnvironmentType.EARTH, ProjectileType.BASEBALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=8.936,
                        time_to_max_height=1.313,
                        dist_at_max_height=16.938,
                        total_flight_time=2.695,
                        total_distance=32.245)
        ),
        (CannedInputs(EnvironmentType.EARTH, ProjectileType.BOWLING_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=9.899,
                        time_to_max_height=1.411,
                        dist_at_max_height=19.536,
                        total_flight_time=2.838,
                        total_distance=38.587)
        ),
        (CannedInputs(EnvironmentType.EARTH, ProjectileType.SHOT_PUT, speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.387,
                        max_possible_flight_time=4.077,
                        max_height=10.110,
                        time_to_max_height=1.431,
                        dist_at_max_height=20.119,
                        total_flight_time=2.871,
                        total_distance=40.142)
        ),
        (CannedInputs(EnvironmentType.MARS, ProjectileType.GOLF_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=107.527,
                        max_possible_height=53.763,
                        max_possible_flight_time=10.753,
                        max_height=26.706,
                        time_to_max_height=3.785,
                        dist_at_max_height=53.274,
                        total_flight_time=7.570,
                        total_distance=106.107)
        ),
        (CannedInputs(EnvironmentType.MARS, ProjectileType.PING_PONG_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=107.527,
                        max_possible_height=53.763,
                        max_possible_flight_time=10.753,
                        max_height=24.546,
                        time_to_max_height=3.559,
                        dist_at_max_height=47.210,
                        total_flight_time=7.258,
                        total_distance=91.229)
        ),
        (CannedInputs(EnvironmentType.MARS, ProjectileType.TENNIS_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=107.527,
                        max_possible_height=53.763,
                        max_possible_flight_time=10.753,
                        max_height=26.537,
                        time_to_max_height=3.763,
                        dist_at_max_height=52.729,
                        total_flight_time=7.548,
                        total_distance=104.900)
        ),
        (CannedInputs(EnvironmentType.MARS, ProjectileType.BASEBALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=107.527,
                        max_possible_height=53.763,
                        max_possible_flight_time=10.753,
                        max_height=26.711,
                        time_to_max_height=3.785,
                        dist_at_max_height=53.281,
                        total_flight_time=7.570,
                        total_distance=106.132)
        ),
        (CannedInputs(EnvironmentType.MARS, ProjectileType.BOWLING_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=107.527,
                        max_possible_height=53.763,
                        max_possible_flight_time=10.753,
                        max_height=26.847,
                        time_to_max_height=3.796,
                        dist_at_max_height=53.629,
                        total_flight_time=7.591,
                        total_distance=107.169)
        ),
        (CannedInputs(EnvironmentType.MARS, ProjectileType.SHOT_PUT, speed=20, angle=45),
        ExpectedResults(max_possible_dist=107.527,
                        max_possible_height=53.763,
                        max_possible_flight_time=10.753,
                        max_height=26.872,
                        time_to_max_height=3.796,
                        dist_at_max_height=53.665,
                        total_flight_time=7.591,
                        total_distance=107.306)
        ),
        (CannedInputs(EnvironmentType.MOON, ProjectileType.GOLF_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=246.914,
                        max_possible_height=123.457,
                        max_possible_flight_time=24.691,
                        max_height=61.728,
                        time_to_max_height=8.741,
                        dist_at_max_height=123.613,
                        total_flight_time=17.457,
                        total_distance=246.876)
        ),
        (CannedInputs(EnvironmentType.MOON, ProjectileType.PING_PONG_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=246.914,
                        max_possible_height=123.457,
                        max_possible_flight_time=24.691,
                        max_height=61.728,
                        time_to_max_height=8.741,
                        dist_at_max_height=123.613,
                        total_flight_time=17.457,
                        total_distance=246.876)
        ),
        (CannedInputs(EnvironmentType.MOON, ProjectileType.TENNIS_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=246.914,
                        max_possible_height=123.457,
                        max_possible_flight_time=24.691,
                        max_height=61.728,
                        time_to_max_height=8.741,
                        dist_at_max_height=123.613,
                        total_flight_time=17.457,
                        total_distance=246.876)
        ),
        (CannedInputs(EnvironmentType.MOON, ProjectileType.BASEBALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=246.914,
                        max_possible_height=123.457,
                        max_possible_flight_time=24.691,
                        max_height=61.728,
                        time_to_max_height=8.741,
                        dist_at_max_height=123.613,
                        total_flight_time=17.457,
                        total_distance=246.876)
        ),
        (CannedInputs(EnvironmentType.MOON, ProjectileType.BOWLING_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=246.914,
                        max_possible_height=123.457,
                        max_possible_flight_time=24.691,
                        max_height=61.728,
                        time_to_max_height=8.741,
                        dist_at_max_height=123.613,
                        total_flight_time=17.457,
                        total_distance=246.876)
        ),
        (CannedInputs(EnvironmentType.MOON, ProjectileType.SHOT_PUT, speed=20, angle=45),
        ExpectedResults(max_possible_dist=246.914,
                        max_possible_height=123.457,
                        max_possible_flight_time=24.691,
                        max_height=61.728,
                        time_to_max_height=8.741,
                        dist_at_max_height=123.613,
                        total_flight_time=17.457,
                        total_distance=246.876)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.GOLF_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=45.096,
                        max_possible_height=22.548,
                        max_possible_flight_time=4.510,
                        max_height=1.955,
                        time_to_max_height=0.501,
                        dist_at_max_height=2.689,
                        total_flight_time=1.294,
                        total_distance=4.044)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.PING_PONG_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=45.096,
                        max_possible_height=22.548,
                        max_possible_flight_time=4.510,
                        max_height=0.227,
                        time_to_max_height=0.144,
                        dist_at_max_height=0.281,
                        total_flight_time=0.437,
                        total_distance=0.385)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.TENNIS_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=45.096,
                        max_possible_height=22.548,
                        max_possible_flight_time=4.510,
                        max_height=1.211,
                        time_to_max_height=0.374,
                        dist_at_max_height=1.602,
                        total_flight_time=1.015,
                        total_distance=2.328)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.BASEBALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=45.096,
                        max_possible_height=22.548,
                        max_possible_flight_time=4.510,
                        max_height=1.991,
                        time_to_max_height=0.510,
                        dist_at_max_height=2.753,
                        total_flight_time=1.308,
                        total_distance=4.133)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.BOWLING_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=45.096,
                        max_possible_height=22.548,
                        max_possible_flight_time=4.510,
                        max_height=4.955,
                        time_to_max_height=0.906,
                        dist_at_max_height=7.768,
                        total_flight_time=2.083,
                        total_distance=12.837)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.SHOT_PUT, speed=20, angle=45),
        ExpectedResults(max_possible_dist=45.096,
                        max_possible_height=22.548,
                        max_possible_flight_time=4.510,
                        max_height=7.925,
                        time_to_max_height=1.245,
                        dist_at_max_height=13.938,
                        total_flight_time=2.661,
                        total_distance=25.008)
        ),
        (CannedInputs(EnvironmentType.JUPITER, ProjectileType.GOLF_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=16.136,
                        max_possible_height=8.068,
                        max_possible_flight_time=1.614,
                        max_height=4.002,
                        time_to_max_height=0.567,
                        dist_at_max_height=7.967,
                        total_flight_time=1.135,
                        total_distance=15.883)
        ),
        (CannedInputs(EnvironmentType.JUPITER, ProjectileType.PING_PONG_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=16.136,
                        max_possible_height=8.068,
                        max_possible_flight_time=1.614,
                        max_height=3.623,
                        time_to_max_height=0.529,
                        dist_at_max_height=6.931,
                        total_flight_time=1.079,
                        total_distance=13.292)
        ),
        (CannedInputs(EnvironmentType.JUPITER, ProjectileType.TENNIS_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=16.136,
                        max_possible_height=8.068,
                        max_possible_flight_time=1.614,
                        max_height=3.972,
                        time_to_max_height=0.565,
                        dist_at_max_height=7.895,
                        total_flight_time=1.131,
                        total_distance=15.665)
        ),
        (CannedInputs(EnvironmentType.JUPITER, ProjectileType.BASEBALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=16.136,
                        max_possible_height=8.068,
                        max_possible_flight_time=1.614,
                        max_height=4.003,
                        time_to_max_height=0.567,
                        dist_at_max_height=7.968,
                        total_flight_time=1.135,
                        total_distance=15.887)
        ),
        (CannedInputs(EnvironmentType.JUPITER, ProjectileType.BOWLING_BALL, speed=20, angle=45),
        ExpectedResults(max_possible_dist=16.136,
                        max_possible_height=8.068,
                        max_possible_flight_time=1.614,
                        max_height=4.028,
                        time_to_max_height=0.571,
                        dist_at_max_height=8.060,
                        total_flight_time=1.139,
                        total_distance=16.075)
        ),
        (CannedInputs(EnvironmentType.JUPITER, ProjectileType.SHOT_PUT, speed=20, angle=45),
        ExpectedResults(max_possible_dist=16.136,
                        max_possible_height=8.068,
                        max_possible_flight_time=1.614,
                        max_height=4.032,
                        time_to_max_height=0.571,
                        dist_at_max_height=8.066,
                        total_flight_time=1.139,
                        total_distance=16.100)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.SHOT_PUT, speed=50, angle=20),
        ExpectedResults(max_possible_dist=281.849,
                        max_possible_height=140.924,
                        max_possible_flight_time=11.274,
                        max_height=7.625,
                        time_to_max_height=1.127,
                        dist_at_max_height=32.867,
                        total_flight_time=2.559,
                        total_distance=53.677)
        ),
        (CannedInputs(EnvironmentType.VENUS, ProjectileType.SHOT_PUT, speed=100, angle=85),
        ExpectedResults(max_possible_dist=1127.396,
                        max_possible_height=563.698,
                        max_possible_flight_time=22.548,
                        max_height=64.704,
                        time_to_max_height=2.864,
                        dist_at_max_height=7.798,
                        total_flight_time=7.779,
                        total_distance=12.286)
        ),
    ], ids=generate_test_id)
    def test_canned(self, inputs: CannedInputs, expected: ExpectedResults) -> None:
        """
        Runs the canned Environment and Projectile tests.
        """
        environment = ENVIRONMENTS[inputs.env]
        projectile = PROJECTILES[inputs.proj]
        sim = ProjectileSimulator(environment, projectile, inputs.speed, inputs.angle)
        sim.run_simulation()
        self.check_results(sim, expected)


    @pytest.mark.parametrize('inputs, expected', [
        (CustomInputs(Environment(EnvironmentType.CUSTOM, gravity=24.79, air_density=0),
                      Projectile(ProjectileType.CUSTOM, mass=1, radius=1),
                      speed=20,
                      angle=45),
         ExpectedResults(max_possible_dist=16.136,
                         max_possible_height=8.068,
                         max_possible_flight_time=1.614,
                         max_height=4.034,
                         time_to_max_height=0.571,
                         dist_at_max_height=8.069,
                         total_flight_time=1.139,
                         total_distance=16.109)
        ),
        (CustomInputs(Environment(EnvironmentType.CUSTOM, gravity=100, air_density=50),
                      Projectile(ProjectileType.CUSTOM, mass=40, radius=0.2),
                      speed=200,
                      angle=20),
        ExpectedResults(max_possible_dist=400.000,
                        max_possible_height=200.000,
                        max_possible_flight_time=4.000,
                        max_height=7.622,
                        time_to_max_height=0.316,
                        dist_at_max_height=30.750,
                        total_flight_time=0.752,
                        total_distance=48.202)
        ),
    ], ids=generate_test_id)
    def test_custom(self, inputs: CustomInputs, expected: ExpectedResults) -> None:
        """
        Runs the custom Environment and Projectile tests.
        """
        environment = inputs.env
        projectile = inputs.proj
        sim = ProjectileSimulator(environment, projectile, inputs.speed, inputs.angle)
        sim.run_simulation()
        self.check_results(sim, expected)


    def test_launch_angle_0(self) -> None:
        """
        Tests too low of launch angle.
        """
        with pytest.raises(ValueError):
            ProjectileSimulator(environment=ENVIRONMENTS[EnvironmentType.EARTH],
                                projectile=PROJECTILES[ProjectileType.GOLF_BALL],
                                speed=20,
                                angle=0)

    def test_launch_angle_91(self) -> None:
        """
        Tests too high of launch angle.
        """
        with pytest.raises(ValueError):
            ProjectileSimulator(environment=ENVIRONMENTS[EnvironmentType.EARTH],
                                projectile=PROJECTILES[ProjectileType.GOLF_BALL],
                                speed=20,
                                angle=91)
