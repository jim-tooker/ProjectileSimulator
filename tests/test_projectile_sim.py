import pytest
from dataclasses import dataclass
from projectile_sim import  ProjectileSimulator, \
                            Environment, Projectile, \
                            ENVIRONMENTS, PROJECTILES, \
                            EnvironmentType, ProjectileType

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
    ProjectileSimulator.disable_gui(True)
    toler = 0.001

    @staticmethod
    def generate_test_id(inputs):
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
        assert sim.max_possible_dist == pytest.approx(expected.max_possible_dist, abs=self.toler)
        assert sim.max_possible_height == pytest.approx(expected.max_possible_height, abs=self.toler)
        assert sim.max_possible_flight_time == pytest.approx(expected.max_possible_flight_time, abs=self.toler)
        assert sim.max_height == pytest.approx(expected.max_height, abs=self.toler)
        assert sim.time_to_max_height == pytest.approx(expected.time_to_max_height, abs=self.toler)
        assert sim.dist_at_max_height == pytest.approx(expected.dist_at_max_height, abs=self.toler)
        assert sim.total_flight_time == pytest.approx(expected.total_flight_time, abs=self.toler)
        assert sim.total_distance == pytest.approx(expected.total_distance, abs=self.toler)


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
    ], ids=generate_test_id)
    def test_canned(self, inputs: CannedInputs, expected: ExpectedResults) -> None:
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
    ], ids=generate_test_id)
    def test_custom(self, inputs, expected):
        environment = inputs.env
        projectile = inputs.proj
        sim = ProjectileSimulator(environment, projectile, inputs.speed, inputs.angle)
        sim.run_simulation()
        self.check_results(sim, expected)


    def test_launch_angle_0(self) -> None:
        with pytest.raises(ValueError):
            ProjectileSimulator(environment=ENVIRONMENTS[EnvironmentType.EARTH],
                                projectile=PROJECTILES[ProjectileType.GOLF_BALL],
                                speed=20,
                                angle=0)

    def test_launch_angle_91(self) -> None:
        with pytest.raises(ValueError):
            ProjectileSimulator(environment=ENVIRONMENTS[EnvironmentType.EARTH],
                                projectile=PROJECTILES[ProjectileType.GOLF_BALL],
                                speed=20,
                                angle=91)
