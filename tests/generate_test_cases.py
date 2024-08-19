import re
import sys
from io import StringIO
import itertools
from typing import List, Tuple

from projectile_sim import  ProjectileSimulator
from environment import Environment, EnvironmentType, ENVIRONMENTS
from projectile import Projectile, ProjectileType, PROJECTILES

def capture_and_format_test_case(env_type: EnvironmentType,
                                 gravity: float,
                                 air_density: float,
                                 proj_type: ProjectileType,
                                 mass: float,
                                 radius: float,
                                 speed: float,
                                 angle: float) -> None:
    # Redirect stdout to capture it
    old_stdout = sys.stdout
    sys.stdout = mystdout = StringIO()

    # Call your main function here
    simulator = ProjectileSimulator(environment=Environment(env_type, gravity, air_density),
                                    projectile=Projectile(proj_type, mass, radius),
                                    speed=speed,
                                    angle=angle)
    simulator.disable_gui(True)
    simulator.run_simulation()

    # Restore stdout
    sys.stdout = old_stdout
    output = mystdout.getvalue()

    # Use regex to extract values
    search_results = {
        'max_possible_dist': re.search(r'Max possible distance @ 45°: (\d+\.\d+)', output),
        'max_possible_height': re.search(r'Max possible height @ 90°: (\d+\.\d+)', output),
        'max_possible_flight_time': re.search(r'Max possible flight time @ 90°: (\d+\.\d+)', output),
        'max_height': re.search(r'Max height: (\d+\.\d+)', output),
        'time_to_max_height': re.search(r'Time to max height: (\d+\.\d+)', output),
        'dist_at_max_height': re.search(r'Distance at max height: (\d+\.\d+)', output),
        'total_flight_time': re.search(r'Total flight time: (\d+\.\d+)', output),
        'total_distance': re.search(r'Total distance: (\d+\.\d+)', output)
    }

    # Fail if search results failed.
    assert search_results['max_possible_dist']
    assert search_results['max_possible_height']
    assert search_results['max_possible_flight_time']
    assert search_results['max_height']
    assert search_results['time_to_max_height']
    assert search_results['dist_at_max_height']
    assert search_results['total_flight_time']
    assert search_results['total_distance']

    values = {
        'max_possible_dist': float(search_results['max_possible_dist'].group(1)),
        'max_possible_height': float(search_results['max_possible_height'].group(1)),
        'max_possible_flight_time': float(search_results['max_possible_flight_time'].group(1)),
        'max_height': float(search_results['max_height'].group(1)),
        'time_to_max_height': float(search_results['time_to_max_height'].group(1)),
        'dist_at_max_height': float(search_results['dist_at_max_height'].group(1)),
        'total_flight_time': float(search_results['total_flight_time'].group(1)),
        'total_distance': float(search_results['total_distance'].group(1)),
    }

    canned_test_case = f"""        (CannedInputs(EnvironmentType.{env_type.name}, ProjectileType.{proj_type.name}, speed={speed}, angle={angle}),
        ExpectedResults(max_possible_dist={values['max_possible_dist']:.3f},
                        max_possible_height={values['max_possible_height']:.3f},
                        max_possible_flight_time={values['max_possible_flight_time']:.3f},
                        max_height={values['max_height']:.3f},
                        time_to_max_height={values['time_to_max_height']:.3f},
                        dist_at_max_height={values['dist_at_max_height']:.3f},
                        total_flight_time={values['total_flight_time']:.3f},
                        total_distance={values['total_distance']:.3f})
        ),"""

    custom_test_case = f"""        (CustomInputs(Environment(EnvironmentType.{env_type.name}, gravity={gravity}, air_density={air_density}),
                      Projectile(ProjectileType.{proj_type.name}, mass={mass}, radius={radius}),
                      speed={speed},
                      angle={angle}),
        ExpectedResults(max_possible_dist={values['max_possible_dist']:.3f},
                        max_possible_height={values['max_possible_height']:.3f},
                        max_possible_flight_time={values['max_possible_flight_time']:.3f},
                        max_height={values['max_height']:.3f},
                        time_to_max_height={values['time_to_max_height']:.3f},
                        dist_at_max_height={values['dist_at_max_height']:.3f},
                        total_flight_time={values['total_flight_time']:.3f},
                        total_distance={values['total_distance']:.3f})
        ),"""
    if env_type != EnvironmentType.CUSTOM:
        print(canned_test_case, end='')
    else:
        print(custom_test_case, end='')


def generate_canned_test_cases(speeds: List[float], angles: List[float]) \
      -> List[Tuple[EnvironmentType, float, float, ProjectileType, float, float, float, float]]:
    """
    Generate test cases for all combinations of canned environments and projectiles.
    """
    test_cases = []
    for env_type, proj_type in itertools.product(ENVIRONMENTS.keys(), PROJECTILES.keys()):
        if env_type != EnvironmentType.CUSTOM:
            for speed, angle in itertools.product(speeds, angles):
                test_cases.append((ENVIRONMENTS[env_type].type,
                                   ENVIRONMENTS[env_type].gravity,
                                   ENVIRONMENTS[env_type].air_density,
                                   PROJECTILES[proj_type].type,
                                   PROJECTILES[proj_type].mass,
                                   PROJECTILES[proj_type].radius,
                                   speed, angle))
    return test_cases

def generate_custom_test_cases(custom_params: List[Tuple[EnvironmentType, float, float, ProjectileType, float, float, float, float]]) \
      -> List[Tuple[EnvironmentType, float, float, ProjectileType, float, float, float, float]]:
    """
    Generate test cases from custom parameters.
    
    custom_params: List of tuples in the format (env_type, gravity, air_density, 
                                                 proj_type, mass, radius, speed, angle)
    """
    test_cases = []
    for params in custom_params:
        env_type, gravity, air_density, proj_type, mass, radius, speed, angle = params
        test_cases.append((env_type, gravity, air_density, proj_type, mass, radius, speed, angle))
    return test_cases

def run_all_test_cases(output_file: str = "trajectory_test_cases.txt") -> None:
    # Define speeds and angles for canned test cases
    speeds: List[float] = [20]  # m/s
    angles: List[float] = [45]  # degrees

    # Generate canned test cases
    canned_cases = generate_canned_test_cases(speeds, angles)

    # Define custom test cases (if any)
    custom_params = [
        (EnvironmentType.CUSTOM, 100.0, 50.0, ProjectileType.CUSTOM, 40.0, 0.2, 200.0, 20.0),
        # Add more custom cases as needed
    ]
    custom_cases = generate_custom_test_cases(custom_params)

    # Combine all test cases
    all_cases = canned_cases + custom_cases

    # Run all test cases and write to file
    with open(output_file, 'w') as f:
        for case in all_cases:
            env_type, gravity, air_density, proj_type, mass, radius, speed, angle = case
            
            # Capture the output as a string
            old_stdout = sys.stdout
            sys.stdout = mystdout = StringIO()
            capture_and_format_test_case(env_type, gravity, air_density, proj_type, mass, radius, speed, angle)
            sys.stdout = old_stdout
            test_case_output = mystdout.getvalue()
            
            # Write to file
            f.write(test_case_output)
            f.write('\n')  # Extra line between test cases

    print(f"Generated {len(all_cases)} test cases in {output_file}")

# Run the function
run_all_test_cases()
