import pytest
from dataclasses import dataclass
from projectile_sim import ProjectileSimulator, Environment, Projectile, ENVIRONMENTS, PROJECTILES

__author__ = "Jim Tooker"


@dataclass
class ExpectedResults:
    """
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
    """
    env: str
    proj: str
    speed: float
    angle: float


@dataclass
class CustomInputs:
    """
    """
    env: Environment
    proj: Projectile
    speed: float
    angle: float


class TestProjectileSimulator:
    ProjectileSimulator.disable_gui(True)
    tolerance = 0.001

    def check_results(self, sim: ProjectileSimulator, expected: ExpectedResults) -> None:
        assert sim.max_possible_dist == pytest.approx(expected.max_possible_dist, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(expected.max_possible_height, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_flight_time == pytest.approx(expected.max_possible_flight_time, rel=TestProjectileSimulator.tolerance)
        assert sim.max_height == pytest.approx(expected.max_height, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(expected.time_to_max_height, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(expected.dist_at_max_height, rel=TestProjectileSimulator.tolerance)
        assert sim.total_flight_time == pytest.approx(expected.total_flight_time, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(expected.total_distance, rel=TestProjectileSimulator.tolerance)


    @pytest.mark.parametrize("inputs, expected", [
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=20, angle=45),
         ExpectedResults(max_possible_dist=40.775,
                         max_possible_height=20.3874,
                         max_possible_flight_time=4.08,
                         max_height=10.20,
                         time_to_max_height=1.443,
                         dist_at_max_height=20.4131,
                         total_flight_time=2.883,
                         total_distance=40.775)
        ),
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=20, angle=1),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.3874,
                        max_possible_flight_time=4.08,
                        max_height=0.0062,
                        time_to_max_height=0.0367,
                        dist_at_max_height=0.734,
                        total_flight_time=0.0693,
                        total_distance=1.386)
        ),
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=20, angle=5),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.3874,
                        max_possible_flight_time=4.08,
                        max_height=0.155,
                        time_to_max_height=0.1794,
                        dist_at_max_height=3.575,
                        total_flight_time=0.3547,
                        total_distance=7.068)
        ),
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=20, angle=90),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.3874,
                        max_possible_flight_time=4.08,
                        max_height=20.3874,
                        time_to_max_height=2.04,
                        dist_at_max_height=0.0,
                        total_flight_time=4.077,
                        total_distance=0.0)
        ),
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=20, angle=88),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.3874,
                        max_possible_flight_time=4.08,
                        max_height=20.38,
                        time_to_max_height=2.04,
                        dist_at_max_height=1.423,
                        total_flight_time=4.073,
                        total_distance=2.843)
        ),
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=1, angle=45),
        ExpectedResults(max_possible_dist=0.1019,
                        max_possible_height=0.051,
                        max_possible_flight_time=0.204,
                        max_height=0.0255,
                        time_to_max_height=0.0727,
                        dist_at_max_height=0.0514,
                        total_flight_time=0.1433,
                        total_distance=0.1013)
        ),
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=5, angle=45),
        ExpectedResults(max_possible_dist=2.55,
                        max_possible_height=1.274,
                        max_possible_flight_time=1.02,
                        max_height=0.637,
                        time_to_max_height=0.3605,
                        dist_at_max_height=1.274,
                        total_flight_time=0.719,
                        total_distance=2.54)
        ),
        (CannedInputs("Earth (No Air)", "Golf Ball", speed=100, angle=45),
        ExpectedResults(max_possible_dist=1019.37,
                        max_possible_height=509.68,
                        max_possible_flight_time=20.387,
                        max_height=254.84,
                        time_to_max_height=7.22,
                        dist_at_max_height=510.33,
                        total_flight_time=14.41,
                        total_distance=1019.37)
        ),
        (CannedInputs("Earth", "Golf Ball", speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.3874,
                        max_possible_flight_time=4.08,
                        max_height=8.91,
                        time_to_max_height=1.31,
                        dist_at_max_height=16.85,
                        total_flight_time=2.69,
                        total_distance=32.1)
        ),
        (CannedInputs("Earth", "Ping Pong Ball", speed=20, angle=45),
        ExpectedResults(max_possible_dist=40.775,
                        max_possible_height=20.3874,
                        max_possible_flight_time=4.08,
                        max_height=3.95,
                        time_to_max_height=0.754,
                        dist_at_max_height=6.044,
                        total_flight_time=1.766,
                        total_distance=9.826)
        ),        
    ])
    def test_canned(self, inputs: CannedInputs, expected: ExpectedResults) -> None:
        environment = ENVIRONMENTS[inputs.env]
        projectile = PROJECTILES[inputs.proj]
        sim = ProjectileSimulator(environment, projectile, inputs.speed, inputs.angle)
        sim.run_simulation()
        self.check_results(sim, expected)


    @pytest.mark.parametrize("inputs, expected", [
        (CustomInputs(Environment("Jupiter (No Air)", gravity=24.79, air_density=0),
                      PROJECTILES["Golf Ball"],
                      speed=20,
                      angle=45),
         ExpectedResults(max_possible_dist=16.136,
                         max_possible_height=8.07,
                         max_possible_flight_time=1.613,
                         max_height=4.03,
                         time_to_max_height=0.571,
                         dist_at_max_height=8.07,
                         total_flight_time=1.14,
                         total_distance=16.109)
        ),
    ])
    def test_custom(self, inputs, expected):
        environment = inputs.env
        projectile = inputs.proj
        sim = ProjectileSimulator(environment, projectile, inputs.speed, inputs.angle)
        sim.run_simulation()
        self.check_results(sim, expected)


    def test_launch_angle_0(self) -> None:
        with pytest.raises(ValueError):
            ProjectileSimulator(environment=Environment("Earth", gravity=9.81, air_density=0),
                                projectile=Projectile("Custom", mass=0, radius=0),
                                speed=20,
                                launch_angle=0)

    def test_launch_angle_91(self) -> None:
        with pytest.raises(ValueError):
            ProjectileSimulator(environment=Environment("Earth", gravity=9.81, air_density=0),
                                projectile=Projectile("Custom", mass=0, radius=0),
                                speed=20,
                                launch_angle=91)

