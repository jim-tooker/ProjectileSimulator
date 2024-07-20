import pytest
from ProjectileSimulator import ProjectileSimulator

class TestProjectileSimulator:
    ProjectileSimulator.disable_gui(True)
    tolerance = 0.01

    def test_20_45(self):
        speed = 20
        angle = 45
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(40.82, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(20.41, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(10.20, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(1.44, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(20.41, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(2.89, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(40.82, rel=TestProjectileSimulator.tolerance)

    def test_20_1(self):
        speed = 20
        angle = 1
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(40.82, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(20.41, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(0.0062, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(0.0356, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(0.71, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(0.0712, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(1.42, rel=TestProjectileSimulator.tolerance)

    def test_20_5(self):
        speed = 20
        angle = 5
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(40.82, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(20.41, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(0.155, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(0.1779, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(3.54, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(0.3558, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(7.09, rel=TestProjectileSimulator.tolerance)

    def test_20_90(self):
        speed = 20
        angle = 90
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(40.82, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(20.41, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(20.41, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(2.04, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(0.0, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(4.08, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(0.0, rel=TestProjectileSimulator.tolerance)

    def test_20_88(self):
        speed = 20
        angle = 88
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(40.82, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(20.41, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(20.38, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(2.04, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(1.42, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(4.08, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(2.85, rel=TestProjectileSimulator.tolerance)

    def test_1_45(self):
        speed = 1
        angle = 45
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(0.1020, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(0.051, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(0.0255, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(0.0722, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(0.051, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(0.1443, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(0.1020, rel=TestProjectileSimulator.tolerance)

    def test_5_45(self):
        speed = 5
        angle = 45
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(2.55, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(1.28, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(0.64, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(0.36, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(1.28, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(0.72, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(2.55, rel=TestProjectileSimulator.tolerance)

    def test_100_45(self):
        speed = 100
        angle = 45
        sim = ProjectileSimulator(speed, angle)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(1020.41, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(510.2, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(255.1, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(7.22, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(510.2, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(14.43, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(1020.41, rel=TestProjectileSimulator.tolerance)

    def test_100_45_Jupiter(self):
        speed = 100
        angle = 45
        gravity = 24.79  # Jupiter gravity
        sim = ProjectileSimulator(speed, angle, gravity)
        sim.run_simulation()
        assert sim.max_possible_dist == pytest.approx(403.39, rel=TestProjectileSimulator.tolerance)
        assert sim.max_possible_height == pytest.approx(201.69, rel=TestProjectileSimulator.tolerance)

        assert sim.max_height == pytest.approx(100.85, rel=TestProjectileSimulator.tolerance)
        assert sim.time_to_max_height == pytest.approx(2.85, rel=TestProjectileSimulator.tolerance)
        assert sim.dist_at_max_height == pytest.approx(201.69, rel=TestProjectileSimulator.tolerance)

        assert sim.total_flight_time == pytest.approx(5.70, rel=TestProjectileSimulator.tolerance)
        assert sim.total_distance == pytest.approx(403.39, rel=TestProjectileSimulator.tolerance)

    def test_launch_angle_0(self):
        speed = 20
        angle = 0
        with pytest.raises(ValueError):
            ProjectileSimulator(speed, angle)

    def test_launch_angle_91(self):
        speed = 20
        angle = 91
        with pytest.raises(ValueError):
            ProjectileSimulator(speed, angle)

