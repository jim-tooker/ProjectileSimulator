"""
This module simulates projectile motion using VPython for visualization.
It provides a ProjectileSimulator class that calculates and displays
the trajectory of a projectile given initial conditions.
"""

import argparse
import math
from typing import Dict, Tuple, List, Optional
import readchar
import vpython as vp


class ProjectileSimulator:
    """
    A class to simulate and visualize projectile motion.

    Attributes:
        speed (float): The initial speed of the projectile (m/s).
        launch_angle (float): The launch angle (degrees).
        gravity (float): The acceleration due to gravity (m/s^2).
        angle_rad (float): The launch angle (radians).
        v0x (float): The velocity component in the x direction (m)
        v0y (float): The velocity component in the y direction (m)
        time_to_max_height (float): The time to reach the maximum height (s)
        total_flight_time (float): The total flight time (s)
        total_distance (float): The total distance traveled (m)
        dist_at_max_height (float): The x distance traveled when at the heightest point (m)
        max_possible_dist (float): The max possible distance traveled at speed given
                                   (launched at 45°) (m)
        max_possible_height (float): The max possible height reached at speed given
                                     (launched at 90°) (m)
    """

    # Flag to indicate whether the GUI should be disabled (True = no GUI)
    _no_gui: bool = False

    def __init__(self, speed: float, launch_angle: float, gravity: float = 9.8):
        """
        Args:
            speed (float): The initial speed of the projectile in m/s.
            launch_angle (float): The launch angle in degrees.
            gravity (float, optional): The acceleration due to gravity in m/s^2. Defaults to 9.8.

        Raises:
            ValueError: If the launch angle is not greater than 0 and less than or 
                        equal to 90°.
        """
        self.speed: float = speed
        self.launch_angle: float = launch_angle
        self.gravity: float = gravity

        if self.launch_angle <= 0 or self.launch_angle > 90:
            raise ValueError("Launch angle must be >0° and <=90°.")

        # Convert angle to radians for trigonometric calculations
        self.angle_rad: float = math.radians(self.launch_angle)

        # Calculate initial velocity components
        self.v0x: float = self.speed * math.cos(self.angle_rad)
        self.v0y: float = self.speed * math.sin(self.angle_rad)

        # Calculate key trajectory parameters
        self.time_to_max_height: float = self.v0y / self.gravity
        self.total_flight_time: float = 2 * self.time_to_max_height
        self.total_distance: float = self.v0x * self.total_flight_time
        self.max_height: float = (self.v0y**2) / (2 * self.gravity)
        self.dist_at_max_height: float = self.total_distance / 2

        # Calculate maximum possible distance and height (used for graph scaling)
        self.max_possible_dist: float = (self.speed**2) / self.gravity
        self.max_possible_height: float = (self.speed**2) / (2 * self.gravity)

        # Initialize visualization attributes
        self._canvas: Optional[vp.canvas] = None
        self._graph: Optional[vp.graph] = None
        self._curve: Optional[vp.gcurve] = None
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
            import vpython.no_notebook as vp_services
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
        self._graph = vp.graph(
            title=f'Projectile Motion (Speed: {self.speed} m/s, Angle: {
                self.launch_angle}°, Gravity: {self.gravity} m/s²)',
            xtitle='Distance (m)', ytitle='Height (m)',
            xmin=0, xmax=self.max_possible_dist,
            ymin=0, ymax=self.max_possible_height,
            width=800, height=400,
            align='right'
        )
        self._curve = vp.gcurve(color=vp.color.red, dot=True,
                            dot_radius=5, dot_color=vp.color.blue)

    def _create_labels(self) -> None:
        """Create labels for displaying simulation information."""
        assert self._canvas

        # Start labels 2 over from left margin (range -10 to 10)
        left_margin: int = -self._canvas.range + 2

        # Start lines at line number 5  (range 10 to -10)
        line_number: int = 5

        ### Create labels for various trajectory parameters  ###
        self._labels['max_possible_dist'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                     text=f'Max Possible Distance @ 45°: {
            self.max_possible_dist:.2f} m',
            height=16, align='left', box=False)

        line_number -= 1

        self._labels['max_possible_height'] = vp.label(pos=vp.vector(left_margin, line_number, 0),
                                                       text=f'Max Possible Height @ 90°: {
            self.max_possible_height:.2f} m',
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

    def _calculate_dt_and_rate(self,
                               flight_time: float,
                               min_steps: int = 50,
                               max_steps: int = 500,
                               target_fps: int = 60) -> Tuple[float, int]:
        """
        Calculate the time step and frame rate for smooth simulation.

        Args:
            flight_time (float): Total flight time of the projectile.
            min_steps (int): Minimum number of simulation steps.
            max_steps (int): Maximum number of simulation steps.
            target_fps (int): Target frames per second for visualization.

        Returns:
            tuple: (dt, rate) where dt is the time step and rate is the frame rate.
        """
        steps: int = min(max(min_steps, int(flight_time * target_fps)), max_steps)
        dt: float = flight_time / steps
        rate: int = min(int(1 / dt), target_fps)
        return dt, rate

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
            dt, rate = self._calculate_dt_and_rate(self.total_flight_time)
            y_prev: float = 0
            max_height_reached: bool = False

            while t <= self.total_flight_time:
                vp.rate(rate)

                # Calculate current position
                x: float = self.v0x * t
                y: float = self.v0y * t - 0.5 * self.gravity * t**2

                assert self._curve

                # Plot new position
                self._curve.plot(x, y)

                # Update flight time label
                self._labels['flight_time'].text = f'Flight Time: {t:.2f} s'

                # Check if max height is reached and update labels accordingly
                if y >= y_prev and not max_height_reached:
                    self._labels['height'].text = f'Height: {y:.2f} m'
                else:
                    max_height_reached = True
                    self._labels['height'].text = f'Max Height: {
                        self.max_height:.2f} m'
                    self._labels['time_to_max_height'].text = \
                        f'Time to Max Height: {self.time_to_max_height:.2f} s'
                    self._labels['dist_at_max_height'].text = \
                        f'Distance at Max Height: {self.dist_at_max_height:.2f} m'

                # Store current y
                y_prev = y

                # Update time
                t += dt

            assert self._curve

            # Plot final point and update final labels
            self._curve.plot(self.total_distance, 0)
            self._labels['flight_time'].text = f'Total Flight Time: {
                self.total_flight_time:.2f} s'
            self._labels['total_dist'].text = f'Total Distance: {
                self.total_distance:.2f} m'
        # Else, no GUI so simulation doesn't run.  Just print results
        else:
            print()
            print(f'Max possible distance @ 45°: {self.max_possible_dist:.2f} m')
            print(f'Max possible height @ 90°: {self.max_possible_height:.2f} m')
            print()
            print(f'Max height: {self.max_height:.2f} m')
            print(f'Time to max height: {self.time_to_max_height:.2f} s')
            print(f'Distance at max height: {self.dist_at_max_height:.2f} m')
            print()
            print(f'Total flight time: {self.total_flight_time:.2f} s')
            print(f'Total distance: {self.total_distance:.2f} m')
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
            - The initial speed of the projectile (m/s)  
            - The angle of launch (degrees)  
            - Optional: The value of gravity (m/s^2) Defaults to 9.8

    * What it does:  
        - Runs the simulation, optionally displaying it in a VPython GUI window.  
        - Waits for a key press to exit if the GUI is enabled.  

    """
    def _get_user_input() -> Tuple[float, float, float]:
        """
        Get user input for the parameters needed for the simulation.

        Returns:
            Tuple[float, float, float]: Parameters needed for simulation 
            (initial speed, degrees of launch, and gravity value).
        """
        def get_float(prompt: str, default_value: Optional[float] = None) -> float:
            while True:
                try:
                    return float(input(prompt))
                except ValueError:
                    if default_value is not None:
                        return default_value
                    else:
                        print("Please enter a valid number.")


        speed: float = get_float("Enter initial speed (m/s): ")
        degrees: float = get_float("Enter angle of launch (degrees): ")
        gravity: float = get_float("Enter gravity (m/s²) or <Enter> for 9.8 m/s²: ", 9.8)

        return (speed, degrees, gravity)

    parser = argparse.ArgumentParser(description='Projectile Simulator')
    parser.add_argument('--test', action='store_true', help='Run with pre-defined test cases')
    parser.add_argument('--no_gui', action='store_true', help='Run without GUI')
    args = parser.parse_args()

    if args.no_gui is True:
        ProjectileSimulator.disable_gui(True)

    if args.test:
        # Setup some test simulations
        simulations: List[Tuple[float, ...]] = [
            (20, 1),  # initial speed (m/s), Angle (degrees)
            (20, 5),
            (20, 90),
            (20, 88),
            (1, 45),
            (5, 45),
            (20, 45),
            (100, 45),
            (100, 45, 24.79),  # Jupiter gravity
            (20, 100),         # Angle beyond bounds
            (20, 0),           # Angle beyond bounds
        ]

        # Run the simulations
        for params in simulations:
            try:
                simulator = ProjectileSimulator(*params)
                simulator.run_simulation()
                print("Press any key to continue...")
                readchar.readkey()
            except ValueError as e:
                print(e)
    else:
        try:
            # Get user input
            speed, degrees, gravity = _get_user_input()
            simulator = ProjectileSimulator(speed, degrees, gravity)
            simulator.run_simulation()
        except ValueError as e:
            print(e)

    if args.no_gui is False:
        print("Press any key to exit...")
        readchar.readkey()
        ProjectileSimulator.quit_simulation()

if __name__ == '__main__':
    main()
