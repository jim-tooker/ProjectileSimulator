"""
This module simulates projectile motion using VPython for visualization.
It provides a ProjectileSimulator class that calculates and displays
the trajectory of a projectile in different environments and for different
projectile types.
"""

import argparse
from copy import copy
import math
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import IntEnum
import readchar
import vpython as vp
from environment import Environment, EnvironmentType, ENVIRONMENTS
from projectile import Projectile, ProjectileType, PROJECTILES

__author__ = "Jim Tooker"


# Constants
DRAG_COEFFICIENT_SPHERE = 0.47


class GraphType(IntEnum):
    """
    Enum to hold the different type of graphs.
    """
    DIST_HEIGHT = 0
    """Distance vs. Height Graph"""
    TIME_HEIGHT = 1
    """Time vs. Height Graph"""
    TIME_DIST = 2
    """Time vs. Distance Graph"""

class CurveType(IntEnum):
    """
    Enum to hold the different type of curves.
    """
    ACTUAL = 0
    """Curve of actual data with all parameters applied"""
    NO_AIR = 1
    """Curve of data with no air resistance"""
    NO_AIR_45 = 2
    """Curve of ideal curve at 45° and no air resistance"""
    NO_AIR_90 = 3
    """Curve of ideal curve at 90° and no air resistance"""

@dataclass
class Graph:
    """
    Data class to hold Graph information.  Every graph has a list of curves.
    """
    graph: vp.graph
    curves: List[vp.gcurve]

@dataclass
class Trajectory:
    """
    Data class to store trajectory information (Position (x, y), Velocity (vx, vy))
    """
    pos: vp.vector
    vel: vp.vector


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


        ### Initialize visualization attributes  ###

        # Canvas for text
        self._canvas: Optional[vp.canvas] = None

        # We will have 3 graphs (if GUI enabled):
        #   Distance (x) vs. Height (y)
        #   Time (x) vs. Height (y)
        #   Time (x) vs. Distance (y)
        # We will have 4 types of curves on each graph (if GUI enabled):
        #   Actual data
        #   Data at given angle with no air resistance
        #   Data at 45° with no air resistance
        #   Data at 90° with no air resistance
        self._graphs: List[Graph] = []

        # Labels for canvas
        self._labels: Dict[str, vp.label] = {}

    def __del__(self) -> None:
        """
        Clean up VPython objects when the simulator is deleted.
        """
        if hasattr(self, '_canvas') and self._canvas:
            self._canvas.delete()
            self._canvas = None

        if hasattr(self, '_graphs') and self._graphs:
            for graph in self._graphs:
                graph.graph.delete()

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

        # Create blank label so this canvas will show first
        vp.label(text='', box=False)

        # Range of canvas labels will be -10 to 10
        self._canvas.range = 10

    def _create_graph_spacer(self) -> None:
        """
        Create an invisible/empty graph for spacing (VPython doesn't
        separate the graphs decently by itself, so this is a workaround).
        """
        spacing_graph = vp.graph(width=1600,
                                 height=20,
                                 align='right',
                                 foreground=vp.color.gray(0.95),
                                 background=vp.color.gray(0.95))
        
        # Has to have a curve
        spacing_curve = vp.gcurve(graph=spacing_graph)

        # Plot a point so graph shows up
        spacing_curve.plot(0,0)

    def _create_canvas_spacer(self) -> None:
        """
        Create blank canvas for spacer between canvas and 1st graph (VPython workaround)
        """
        vp.canvas(width=20, height=400, align='left', background=vp.color.gray(0.95))

        # We need to create a label for canvas to show up
        vp.label(text='', box=False)

        # Reselect the "real" canvas, so this one won't be used
        # (by default VPyhon uses the last canvas created)
        assert self._canvas
        self._canvas.select()

    def _add_graph_legend(self, curves: List[vp.gcurve]) -> None:
        """
        Adds a legend to the specified curves.

        Args:
            curves (List[vp.gcurve]): A list of curves to add a legend to.
        """
        curves[CurveType.ACTUAL].label = f'{self.angle:.1f}°, Actual'
        curves[CurveType.NO_AIR].label = f'{self.angle:.1f}°, No air'
        curves[CurveType.NO_AIR_45].label = '45°, No air'
        curves[CurveType.NO_AIR_90].label = '90°, No air'

    def _setup_graph(self) -> None:
        """Set up the VPython graph for trajectory plotting."""
        # Scaling factor to give some margins to x and y axis
        scale_factor: float = 1.05
        graph_width: int = 800
        graph_height: int = 400

        # Create a space between the canvas and the 1st graph
        self._create_canvas_spacer()

        # Setup main graph
        main_graph = vp.graph(
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
            width=graph_width, height=graph_height,
            align='left'
        )

        # The size of the dot at the end of the curves
        dot_radius: int = 3

        # Create list of main graph curves
        main_curves: List[vp.gcurve] = []

        # Curve for data plot
        main_curves.append(vp.gcurve(color=vp.color.red, dot=True, dot_radius=dot_radius, dot_color=vp.color.red))

        # Plot a point to make it the first graph (first graph to plot a point becomes the first graph in VPython)
        main_curves[CurveType.ACTUAL].plot(0,0)

        # Ideal curve at given angle, no air
        main_curves.append(vp.gcurve(color=vp.color.orange, dot=True, dot_radius=dot_radius, dot_color=vp.color.orange))

        # Ideal curve at 45°, no air
        main_curves.append(vp.gcurve(color=vp.color.blue, dot=True, dot_radius=dot_radius, dot_color=vp.color.blue))

        # Ideal curve at 90°, no air
        main_curves.append(vp.gcurve(color=vp.color.magenta, dot=True, dot_radius=dot_radius, dot_color=vp.color.magenta))

        # Add legend text
        self._add_graph_legend(main_curves)

        # Add Graph and curves to Distance vs. Height graph
        self._graphs.append(Graph(main_graph, main_curves))

        # Insert a graph spacer
        self._create_graph_spacer()

        # Create list of Time vs. Height graph curves
        th_curves: List[vp.gcurve] = []

        # Time vs. Height graph
        th_graph = vp.graph(
            title='Time vs. Height',
            xtitle='Time (s)', ytitle='Height (m)',
            width=graph_width, height=graph_height,
            align='left',
            xmin=0, xmax=self.max_possible_flight_time * scale_factor,
            ymin=0, ymax=self.max_possible_height * scale_factor,
        )
        th_curves.append(vp.gcurve(color=vp.color.red, dot=True, dot_radius=dot_radius, dot_color=vp.color.red))
        th_curves.append(vp.gcurve(color=vp.color.orange, dot=True, dot_radius=dot_radius, dot_color=vp.color.orange))
        th_curves.append(vp.gcurve(color=vp.color.blue, dot=True, dot_radius=dot_radius, dot_color=vp.color.blue))
        th_curves.append(vp.gcurve(color=vp.color.magenta, dot=True, dot_radius=dot_radius, dot_color=vp.color.magenta))

        # Add legend text
        self._add_graph_legend(th_curves)

        # Add Graph and curves to Time vs. Height graph
        self._graphs.append(Graph(th_graph, th_curves))

        # Create list of Time vs. Distance graph curves
        td_curves: List[vp.gcurve] = []

        # Time vs. Distance graph
        td_graph = vp.graph(
            title='Time vs. Distance',
            xtitle='Time (s)', ytitle='Distance (m)',
            width=graph_width, height=graph_height,
            align='right',
            xmin=0, xmax=self.max_possible_flight_time * scale_factor,
            ymin=0, ymax=self.max_possible_dist * scale_factor,
        )
        td_curves.append(vp.gcurve(color=vp.color.red, dot=True, dot_radius=dot_radius, dot_color=vp.color.red))
        td_curves.append(vp.gcurve(color=vp.color.orange, dot=True, dot_radius=dot_radius, dot_color=vp.color.orange))
        td_curves.append(vp.gcurve(color=vp.color.blue, dot=True, dot_radius=dot_radius, dot_color=vp.color.blue))
        td_curves.append(vp.gcurve(color=vp.color.magenta, dot=True, dot_radius=dot_radius, dot_color=vp.color.magenta))

        # Add legend text
        self._add_graph_legend(td_curves)

        # Add Graph and curves to Time vs. Distance graph
        self._graphs.append(Graph(td_graph, td_curves))

    def _create_labels(self) -> None:
        """Create labels for displaying simulation information."""
        assert self._canvas

        # Start labels 1 over from left margin (range -10 to 10)
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

    def _velocity_verlet_update(self, traj: Trajectory, dt: float) -> None:
        """
        Uses the Velocity Verlet algorithm to update the x,y positions and velocities.

        Args:
            traj (Trajectory): The current trajectory of the object (x, y, vx, vy)
            dt (float): Time delta since last update
        """
        # Half-step velocity update
        ax, ay = self._acceleration(traj.vel.x, traj.vel.y)
        vx_half = traj.vel.x + 0.5 * ax * dt
        vy_half = traj.vel.y + 0.5 * ay * dt

        # Full position update
        traj.pos.x = traj.pos.x + vx_half * dt
        traj.pos.y = traj.pos.y + vy_half * dt

        # Recalculate acceleration at new position
        ax_new, ay_new = self._acceleration(vx_half, vy_half)

        # Full velocity update
        traj.vel.x = traj.vel.x + 0.5 * (ax + ax_new) * dt
        traj.vel.y = traj.vel.y + 0.5 * (ay + ay_new) * dt

    def _plot_points(self, curve: Optional[vp.gcurve], x: float, y: float) -> None:
        """
        Plots the x and y points on the curve, if GUI is active

        Args:
            x (float): x coordinate
            y (float): y coordinate
        """
        if curve:
            curve.plot(x, y)

    def _plot_point_on_each_graph(self, curve: CurveType, pos: vp.vector, t: float) -> None:
        """
        Plots the point on the curve specified on all graphs.

        Args:
            curve (CurveType): The curve to plot the data on
            pos (vp.vector): x-y position
            t (float): time
        """
        self._plot_points(self._graphs[GraphType.DIST_HEIGHT].curves[curve], pos.x, pos.y)
        self._plot_points(self._graphs[GraphType.TIME_HEIGHT].curves[curve], t, pos.y)
        self._plot_points(self._graphs[GraphType.TIME_DIST].curves[curve], t, pos.x)

    def _plot_and_update_ideals(self, trajectories: List[Trajectory], t: float) -> None:
        """
        Plots the x, y points on all the graphs for all the ideal curves (no air resistance), then updates the
        ideal x,y positions.

        Args:
            trajectories (List[Trajectory]):  The trajectory data for all the ideal curves
            t (float): time
        """
        for i, _ in enumerate(self._graphs, 1):
            if trajectories[i].pos.y >= 0:
                # Plot points
                self._plot_point_on_each_graph(CurveType(i), trajectories[i].pos, t)

                # Update x and y
                trajectories[i].pos.x = trajectories[i].vel.x * t
                trajectories[i].pos.y = (trajectories[i].vel.y * t) - (0.5 * self.environment.gravity * t**2)

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
        dt: float
        rate: float
        dt, rate = self._calculate_dt_and_rate()
        pos_prev: vp.vector = vp.vector(0, 0, 0)
        max_height_reached: bool = False
        speed_at_45: float = self.speed / math.sqrt(2)

        ## Set initial trajectories for each curve type ##
        trajectories: List[Trajectory] = []

        # Both actual and "actual with no air" start with the initial velocity given (broken into their x-y components)
        trajectories.append(Trajectory(pos=vp.vector(0,0,0), vel=vp.vector(self.v0x, self.v0y, 0)))
        trajectories.append(Trajectory(pos=vp.vector(0,0,0), vel=vp.vector(self.v0x, self.v0y, 0)))

        # At 45°, both x and y velocities are the same
        trajectories.append(Trajectory(pos=vp.vector(0,0,0), vel=vp.vector(speed_at_45, speed_at_45, 0)))

        # At 90°, x velocity component is 0, and y is the full speed given
        trajectories.append(Trajectory(pos=vp.vector(0,0,0), vel=vp.vector(0, self.speed, 0)))

        # While y is above the x-axis for the actual curve
        while trajectories[CurveType.ACTUAL].pos.y >= 0:
            if ProjectileSimulator._no_gui is False:
                vp.rate(rate)

                # Plot all the ideal values on all the graphs
                self._plot_and_update_ideals(trajectories, t)

                # Plot actual x,y position on all the graphs
                self._plot_point_on_each_graph(CurveType.ACTUAL, trajectories[CurveType.ACTUAL].pos, t)

                # Update flight time label
                self._update_label('flight_time', f'Flight Time: {t:.3f} s')

            # Check if max height is reached and update labels accordingly
            if max_height_reached is False:
                if trajectories[CurveType.ACTUAL].pos.y >= pos_prev.y:
                    self._update_label('height', f'Height: {trajectories[CurveType.ACTUAL].pos.y:.3f} m')
                else:
                    max_height_reached = True
                    self.max_height = pos_prev.y
                    self.time_to_max_height = t - dt
                    self.dist_at_max_height = pos_prev.x
                    self._update_label('height', f'Max Height: {self.max_height:.3f} m')
                    self._update_label('time_to_max_height', f'Time to Max Height: {self.time_to_max_height:.3f} s')
                    self._update_label('dist_at_max_height', f'Distance at Max Height: {self.dist_at_max_height:.3f} m')

            # Store current x and y
            pos_prev = copy(trajectories[CurveType.ACTUAL].pos)

            # Update actual x,y position and velocity
            self._velocity_verlet_update(trajectories[CurveType.ACTUAL], dt)

            # Update time
            t += dt

        # Set final values to last know values before y crossed x-axis
        self.total_distance = pos_prev.x
        self.total_flight_time = t - dt

        # If GUI active
        if ProjectileSimulator._no_gui is False:
            # Update final labels
            self._update_label('flight_time', f'Total Flight Time: {self.total_flight_time:.3f} s')
            self._update_label('total_dist', f'Total Distance: {self.total_distance:.3f} m')

            # Continue to complete ideal curve plots if they are in progress still
            while trajectories[CurveType.NO_AIR].pos.y >= 0 or \
                  trajectories[CurveType.NO_AIR_45].pos.y >= 0 or \
                  trajectories[CurveType.NO_AIR_90].pos.y >= 0:

                vp.rate(rate)

                # Plot all the ideal values on all the graphs
                self._plot_and_update_ideals(trajectories, t)

                # Update time
                t += dt
        # Else, print results if no GUI
        else:
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
            simulator = ProjectileSimulator(environment=Environment(EnvironmentType.CUSTOM, gravity=10, air_density=1),
                                            projectile=Projectile(ProjectileType.CUSTOM, mass=10, radius=1),
                                            speed=30,
                                            angle=80)
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
