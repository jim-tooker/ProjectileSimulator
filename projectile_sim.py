"""
This module simulates projectile motion using VPython for visualization.
It provides a ProjectileSimulator class that calculates and displays
the trajectory of a projectile in different environments and for different
projectile types.
"""

import argparse
import math
from typing import Dict, Tuple, Optional
import readchar
import vpython as vp
from environment import Environment, EnvironmentType, ENVIRONMENTS
from projectile import Projectile, ProjectileType, PROJECTILES

__author__ = "Jim Tooker"


# Constants
DRAG_COEFFICIENT_SPHERE = 0.47


class ProjectileSimulator:
    """
    A class to simulate and visualize projectile motion.

    Attributes:
        environment (Environment): The type of environment for simulation.
        projectile (Projectile): The type of projectile for simulation.
        speed (float): The initial speed of the projectile (m/s).
        angle (float): The launch angle (degrees).
        angle_rad (float): The launch angle (radians).
        v0x (float): The velocity component in the x direction (m)
        v0y (float): The velocity component in the y direction (m)
        time_to_max_height (float): The time to reach the maximum height (s)
        total_flight_time (float): The total flight time (s)
        total_distance (float): The total distance traveled (m)
        max_height (float): The maximum height achieved (m)
        dist_at_max_height (float): The x distance traveled when at the highest point (m)
        max_possible_dist (float): The max possible distance traveled at speed given
                                   (launched at 45°) (m)
        max_possible_height (float): The max possible height reached at speed given
                                     (launched at 90°) (m)
        max_possible_flight_time (float): The max possible flight time at speed given
                                          (launched at 90°) (s)
    """

    # Flag to indicate whether the GUI should be disabled (True = no GUI)
    _no_gui: bool = False

    def __init__(self,
                 environment: Environment,
                 projectile: Projectile,
                 speed: float,
                 angle: float):
        """
        Args:
            environment (Environment): The type of environment for simulation.
            projectile (Projectile): The type of projectile for simulation.
            speed (float): The initial speed of the projectile.
            angle (float): The angle of launch.

        Raises:
            ValueError: If the launch angle is not > 0° and <= 90°.
        """
        self.environment: Environment = environment
        self.projectile: Projectile = projectile
        self.speed: float = speed
        self.angle: float = angle

        if self.angle <= 0 or self.angle > 90:
            raise ValueError("Launch angle must be > 0° and <= 90°.")

        # Convert angle to radians for trigonometric calculations
        self.angle_rad: float = math.radians(self.angle)

        # Calculate initial velocity components
        self.v0x: float = self.speed * math.cos(self.angle_rad)
        self.v0y: float = self.speed * math.sin(self.angle_rad)

        # Key trajectory parameters
        self.time_to_max_height: float = 0
        self.total_flight_time: float = 0
        self.total_distance: float = 0
        self.max_height: float = 0
        self.dist_at_max_height: float = 0

        # Calculate maximum possible distance, height, and flight time (used for graph scaling)
        self.max_possible_dist: float = (self.speed**2) / self.environment.gravity
        self.max_possible_height: float = (self.speed**2) / (2 * self.environment.gravity)
        self.max_possible_flight_time: float = 2 * self.speed / self.environment.gravity


        # Initialize visualization attributes
        self._canvas: Optional[vp.canvas] = None
        self._graph: Optional[vp.graph] = None
        self._curve: Optional[vp.gcurve] = None
        self._ideal_curve: Optional[vp.gcurve] = None
        self._labels: Dict[str, vp.label] = {}

    def __del__(self) -> None:
        """
        Clean up VPython objects when the simulator is deleted.
        """
        try:
            if self._canvas:
                self._canvas.delete()
                self._canvas = None

            if self._graph:
                self._graph.delete()
                self._graph = None
        except Exception:
            pass

    @staticmethod
    def quit_simulation() -> None:
        """Stop the VPython server."""
        if ProjectileSimulator._no_gui is False:
            # We don't import vp_services until needed, because importing it will start
            # the server, if not started already.
            import vpython.no_notebook as vp_services  # type: ignore[import-untyped]
            vp_services.stop_server()

    @classmethod
    def disable_gui(cls, no_gui: bool) -> None:
        """
        Enables or disables the GUI.

        Args:
            no_gui (bool): Flag to indicate where GUI should be disabled (True = disable GUI).
        """
        cls._no_gui = no_gui

    def _setup_canvas(self) -> None:
        """Set up the VPython canvas for 3D visualization."""
        self._canvas = vp.canvas(width=400, height=400, align='left')

        # Range of canvas labels will be -10 to 10
        self._canvas.range = 10

    def _setup_graph(self) -> None:
        """Set up the VPython graph for trajectory plotting."""
        # Scaling factor to give some margins to x and y axis
        scale_factor: float = 1.05

        # Setup graph
        self._graph = vp.graph(
            title='<i>Projectile Motion Simulator</i>\n' +
                  f'Initial Speed: {self.speed:.1f} m/s,  Launch Angle: {self.angle:.1f}°\n' +
                  f'Environment: {str(self.environment.type)} ' +
                  f'(Gravity: {self.environment.gravity:.3f} m/s²,  ' +
                  f'Air Density: {self.environment.air_density:.3f} kg/m³)\n' +
                  f'Projectile: {str(self.projectile.type)} ' +
                  f'(Mass: {self.projectile.mass:.3f} kg,  ' +
                  f'Surface Area: {self.projectile.area:.3f} m²)',
            xtitle='Distance (m)', ytitle='Height (m)',
            xmin=0, xmax=self.max_possible_dist * scale_factor,
            ymin=0, ymax=self.max_possible_height * scale_factor,
            width=800, height=400,
            align='right'
        )

        # Curve for data plot
        self._curve = vp.gcurve(color=vp.color.red, dot=True, dot_radius=5, dot_color=vp.color.blue)

        # Ideal curve
        self._ideal_curve = vp.gcurve(color=vp.color.green, dot=True, dot_radius=5, dot_color=vp.color.blue)

        # Add legend text
        self._curve.label = 'Actual'
        self._ideal_curve.label = 'Ideal (45°, No air)'

    def _create_labels(self) -> None:
        """Create labels for displaying simulation information."""
        assert self._canvas

        # Start labels 2 over from left margin (range -10 to 10)
        left_margin: int = -self._canvas.range + 1

        # Start lines at line number 5  (range 10 to -10)
        line_number: int = 5

        ### Create labels for various trajectory parameters  ###
        self._labels['max_possible_dist'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                     text=f'Max Possible Distance (no air) @ 45°: {
                                                     self.max_possible_dist:.3f} m',
                                                     height=16, align='left', box=False)

        line_number -= 1

        self._labels['max_possible_height_45'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                          text=f'Max Possible Height (no air) @ 45°: {
                                                          self.max_possible_height/2:.3f} m',
                                                          height=16, align='left', box=False)

        line_number -= 1

        self._labels['max_possible_height_90'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                          text=f'Max Possible Height (no air) @ 90°: {
                                                          self.max_possible_height:.3f} m',
                                                          height=16, align='left', box=False)

        line_number -= 1

        self._labels['max_possible_flight_time_45'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                               text=f'Max Possible Flight Time (no air) @ 45°: {
                                                               self.max_possible_flight_time/math.sqrt(2):.3f} s',
                                                               height=16, align='left', box=False)

        line_number -= 1

        self._labels['max_possible_flight_time_90'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                               text=f'Max Possible Flight Time (no air) @ 90°: {
                                                               self.max_possible_flight_time:.3f} s',
                                                               height=16, align='left', box=False)

        line_number -= 2

        self._labels['height'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                          text='', height=16, align='left', box=False)

        line_number -= 1

        self._labels['time_to_max_height'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                      text='', height=16, align='left', box=False)

        line_number -= 1

        self._labels['dist_at_max_height'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                      text='', height=16, align='left', box=False)

        line_number -= 2

        self._labels['flight_time'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                               text='', height=16, align='left', box=False)

        line_number -= 1

        self._labels['total_dist'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                              text='', height=16, align='left', box=False)

    def _update_label(self, key: str, text: str) -> None:
        """
        Updates given label's text, if GUI is enabled

        Args:
            key (str): The key for the label dict entry.
            text (str): The str to update the label with
        """
        if ProjectileSimulator._no_gui is False:
            self._labels[key].text = text

    def _calculate_dt_and_rate(self,
                               min_steps: int = 100,
                               max_steps: int = 1000,
                               target_fps: int = 500) -> Tuple[float, int]:
        """
        Calculate the time step and frame rate for smooth simulation.

        Args:
            min_steps (int): Minimum number of simulation steps.
            max_steps (int): Maximum number of simulation steps.
            target_fps (int): Target frames per second for visualization.

        Returns:
            Tuple[float, int]: The time step (dt) and frame rate (rate).
        """
        steps: int = min(max(min_steps, int(self.max_possible_flight_time * target_fps)), max_steps)
        dt: float = self.max_possible_flight_time / steps
        rate: int = min(int(1 / dt), target_fps)
        return dt, rate

    def _acceleration(self, vx: float, vy: float) -> Tuple[float, float]:
        """
        Calculates the acceleration in the x and y direction.

        Args:
            vx (float): x velocity
            vy (float): y velocity

        Returns:
            Tuple[float, float]: The x-y components of acceleration.
        """
        # Calculate speed
        speed = math.sqrt(vx**2 + vy**2)

        # Calculate drag force
        drag_force = 0.5 * self.environment.air_density * speed**2 * DRAG_COEFFICIENT_SPHERE * self.projectile.area

        # Calculate drag acceleration
        if self.projectile.mass > 0:
            drag_acc = drag_force / self.projectile.mass
        else:
            drag_acc = 0

        # Calculate x and y components of acceleration
        ax = -drag_acc * vx / speed
        ay = -self.environment.gravity - (drag_acc * vy / speed)

        return ax, ay

    def _velocity_verlet_update(self, x: float, y: float, vx: float, vy: float, dt: float) \
                                -> Tuple[float, float, float, float]:
        """
        Uses the Velocity Verlet algorithm to update the x,y positions and velocities.

        Args:
            x (float): x position
            y (float): y position
            vx (float): x velocity
            vy (float): y velocity
            dt (float): time delta

        Returns:
            Tuple[float, float, float, float]: The updated x-y positions and velocities (x, y, vx, vy).
        """
        # Half-step velocity update
        ax, ay = self._acceleration(vx, vy)
        vx_half = vx + 0.5 * ax * dt
        vy_half = vy + 0.5 * ay * dt

        # Full position update
        x_new = x + vx_half * dt
        y_new = y + vy_half * dt

        # Recalculate acceleration at new position
        ax_new, ay_new = self._acceleration(vx_half, vy_half)

        # Full velocity update
        vx_new = vx + 0.5 * (ax + ax_new) * dt
        vy_new = vy + 0.5 * (ay + ay_new) * dt

        return x_new, y_new, vx_new, vy_new

    def _plot_points(self, x: float, y: float) -> None:
        """
        Plots the x and y points on the curve if GUI is active

        Args:
            x (float): x coordinate
            y (float): y coordinate
        """
        if self._curve:
            self._curve.plot(x, y)

    def _plot_ideal_points(self, x: float, y: float) -> None:
        """
        Plots the x and y points on the ideal curve if GUI is active

        Args:
            x (float): x coordinate
            y (float): y coordinate
        """
        if self._ideal_curve:
            self._ideal_curve.plot(x, y)

    def run_simulation(self) -> None:
        """
        Run the projectile motion simulation and visualize the results.

        This method sets up the visualization, calculates the projectile's
        trajectory, and updates the display in real-time.
        """
        if ProjectileSimulator._no_gui is False:
            self._setup_canvas()
            self._setup_graph()
            self._create_labels()

        t: float = 0
        x: float = 0
        y: float = 0
        ideal_x: float = 0
        ideal_y: float = 0
        vx: float = self.v0x
        vy: float = self.v0y
        ideal_vx: float = self.speed / math.sqrt(2)
        ideal_vy: float = self.speed / math.sqrt(2)
        dt, rate = self._calculate_dt_and_rate()
        x_prev: float = 0
        y_prev: float = 0
        max_height_reached: bool = False

        # While y is above the x-axis
        while y >= 0:
            if ProjectileSimulator._no_gui is False:
                vp.rate(rate)

            # Plot position
            self._plot_points(x, y)

            # Plot ideal position
            self._plot_ideal_points(ideal_x, ideal_y)

            # Update ideal points
            ideal_x = ideal_vx * t
            ideal_y = (ideal_vy * t) - (0.5 * self.environment.gravity * t**2)

            # Update flight time label
            self._update_label('flight_time', f'Flight Time: {t:.3f} s')

            # Check if max height is reached and update labels accordingly
            if max_height_reached is False:
                if y >= y_prev:
                    self._update_label('height', f'Height: {y:.3f} m')
                else:
                    max_height_reached = True
                    self.max_height = y_prev
                    self.time_to_max_height = t - dt
                    self.dist_at_max_height = x_prev
                    self._update_label('height', f'Max Height: {self.max_height:.3f} m')
                    self._update_label('time_to_max_height', f'Time to Max Height: {self.time_to_max_height:.3f} s')
                    self._update_label('dist_at_max_height', f'Distance at Max Height: {self.dist_at_max_height:.3f} m')

            # Store current x and y
            x_prev = x
            y_prev = y

            # Update position and velocity
            x, y, vx, vy = self._velocity_verlet_update(x, y, vx, vy, dt)

            # Update time
            t += dt

        # Set final values to last know values before y crossed x-axis
        self.total_distance = x_prev
        self.total_flight_time = t - dt

        # Continue to complete ideal curve plot if more time needed
        while ideal_y >= 0:
            if ProjectileSimulator._no_gui is False:
                vp.rate(rate)

            # Plot ideal position
            self._plot_ideal_points(ideal_x, ideal_y)

            # Update ideal points
            ideal_x = ideal_vx * t
            ideal_y = (ideal_vy * t) - (0.5 * self.environment.gravity * t**2)

            # Update time
            t += dt

        # Update final labels
        self._update_label('flight_time', f'Total Flight Time: {self.total_flight_time:.3f} s')
        self._update_label('total_dist', f'Total Distance: {self.total_distance:.3f} m')

        # Print results if no GUI
        if ProjectileSimulator._no_gui is True:
            print()
            print('Input Parameters:')
            print(f'  Initial Speed: {self.speed:.1f} m/s, Launch Angle: {self.angle:.1f}°')
            print(f'  Environment: {str(self.environment.type)}', end=' ')
            print(f'(Gravity: {self.environment.gravity:.3f} m/s²,', end=' ')
            print(f'Air Density: {self.environment.air_density:.3f} kg/m³)')
            print(f'  Projectile: {str(self.projectile.type)}', end=' ')
            print(f'(Mass: {self.projectile.mass:.3f} kg,', end=' ')
            print(f'Surface Area: {self.projectile.area:.3f} m²)')
            print()
            print(f'Max possible distance @ 45°: {self.max_possible_dist:.3f} m')
            print(f'Max possible height @ 45°: {self.max_possible_height/2:.3f} m')
            print(f'Max possible height @ 90°: {self.max_possible_height:.3f} m')
            print(f'Max possible flight time @ 45°: {self.max_possible_flight_time/math.sqrt(2):.3f} s')
            print(f'Max possible flight time @ 90°: {self.max_possible_flight_time:.3f} s')
            print()
            print(f'Max height: {self.max_height:.3f} m')
            print(f'Time to max height: {self.time_to_max_height:.3f} s')
            print(f'Distance at max height: {self.dist_at_max_height:.3f} m')
            print()
            print(f'Total flight time: {self.total_flight_time:.3f} s')
            print(f'Total distance: {self.total_distance:.3f} m')
            print()


def main() -> None:
    """
    Main entry point for the Projectile Simulator.

    This function parses command-line arguments to determine whether to use predefined 
    test parameters or prompt the user for input. It initializes the simulator with the
    appropriate parameters and runs the simulation. If the `--no_gui` flag is set, the
    simulation runs without a graphical user interface (GUI).

    * Command-line Arguments:  
        `--test`: Run the simulation with predefined test parameters.  
        `--no_gui`: Run the simulation without the GUI.  

    * Prompts:  
        - If not using predefined test parameters, the user is prompted to enter:  
            - The Environment (choose from canned Environments, or Custom)  
                * If a Custom Environment is chosen, enter the gravity (m/s²) and air density (kg/m³)  
            - The Projectile (choose from canned Projectiles, or Custom)  
                * If a Custom Projectile is chosen, enter the mass (kg) and radius (m)  
            - The initial speed of the projectile (m/s)  
            - The angle of launch (degrees)  

    * What it does:  
        - Runs the simulation, optionally displaying it in a VPython GUI window.  
        - Waits for a key press to exit if the GUI is enabled.  

    """
    def _get_user_input() -> Tuple[Environment, Projectile, float, float]:
        """
        Get user input for the parameters needed for the simulation.

        Returns:
            Tuple[Environment, Projectile, float, float]]: Parameters needed for simulation 
              (environment type, projectile type, initial speed, angle of launch).
        """
        def _get_float(prompt: str,
                       default_value: Optional[float] = None,
                       min_value: float = float('-inf'),
                       max_value: float = float('inf')) -> float:
            while True:
                try:
                    value = float(input(prompt))
                    if min_value <= value <= max_value:
                        return value
                    else:
                        print(f"Please enter a value between {min_value} and {max_value}.")
                except ValueError:
                    if default_value is not None:
                        return default_value
                    else:
                        print("Please enter a valid number.")


        def _get_environment_choice() -> Environment:
            print("\nSelect an environment:")
            for i, (_, env) in enumerate(ENVIRONMENTS.items()):
                print(f"[{i+1}] {env.display_name}: (Gravity: {env.gravity:.3f} m/s², Air Density: {env.air_density:.3f} kg/m³)")
            print(f"[{len(ENVIRONMENTS)+1}] Custom")

            while True:
                try:
                    choice = int(input("Enter your choice: "))
                    if 1 <= choice <= len(ENVIRONMENTS):
                        return list(ENVIRONMENTS.values())[choice - 1]
                    elif choice == len(ENVIRONMENTS)+1:
                        gravity = _get_float("Enter gravity (m/s²): ")
                        air_density = _get_float("Enter air density (kg/m³): ")
                        return Environment(EnvironmentType.CUSTOM, gravity, air_density)
                    else:
                        raise ValueError
                except ValueError:
                    print("Invalid choice. Please enter a number from the list.")

        def _get_projectile_choice() -> Projectile:
            print("\nSelect a projectile:")
            for i, (_, proj) in enumerate(PROJECTILES.items()):
                print(f"[{i+1}] {proj.display_name}: (Mass: {proj.mass:.3f} kg, Radius: {proj.radius:.3f} m)")
            print(f"[{len(PROJECTILES)+1}] Custom")

            while True:
                try:
                    choice = int(input("Enter your choice: "))
                    if 1 <= choice <= len(PROJECTILES):
                        return list(PROJECTILES.values())[choice - 1]
                    elif choice == len(PROJECTILES)+1:
                        mass = _get_float("Enter mass (kg): ")
                        radius = _get_float("Enter radius (m): ")
                        return Projectile(ProjectileType.CUSTOM, mass, radius)
                    else:
                        raise ValueError
                except ValueError:
                    print("Invalid choice. Please enter a number from the list.")

        environment = _get_environment_choice()
        projectile = _get_projectile_choice()
        speed = _get_float("Enter initial speed (m/s): ")
        angle = _get_float("Enter angle of launch (degrees): ")

        return environment, projectile, speed, angle

    parser = argparse.ArgumentParser(description='Projectile Simulator')
    parser.add_argument('--test', action='store_true', help='Run with pre-defined test cases')
    parser.add_argument('--no_gui', action='store_true', help='Run without GUI')
    args = parser.parse_args()

    if args.no_gui is True:
        ProjectileSimulator.disable_gui(True)

    if args.test:
        # Run the simulation
        try:
            simulator = ProjectileSimulator(environment=ENVIRONMENTS[EnvironmentType.EARTH],
                                            projectile=PROJECTILES[ProjectileType.GOLF_BALL],
                                            speed=10,
                                            angle=45)
            simulator.run_simulation()
        except ValueError as e:
            print(e)
    else:
        try:
            # Get user input
            environment, projectile, speed, angle = _get_user_input()
            simulator = ProjectileSimulator(environment, projectile, speed, angle)
            simulator.run_simulation()
        except ValueError as e:
            print(e)

    if args.no_gui is False:
        print("Press any key to exit...")
        readchar.readkey()
        ProjectileSimulator.quit_simulation()

if __name__ == '__main__':
    main()
