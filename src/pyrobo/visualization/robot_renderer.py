"""
3D robot visualization and animation using matplotlib.

Provides a simple line-based renderer for animating the 3-DOF RRR manipulator.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ..kinematics import forward_kinematics


class RobotRenderer:
    """
    3D renderer for the 3-DOF RRR manipulator with line-based visualization.
    
    Provides methods to initialize and update a 3D visualization with simple lines.
    """
    
    def __init__(self, L, axis_limits=None, figure=None):
        """
        Initialize the robot renderer.
        
        Parameters
        ----------
        L : float
            Link length parameter
        axis_limits : array_like (6,), optional
            Axis limits [xmin, xmax, ymin, ymax, zmin, zmax]
            If None, uses [-4, 4, -4, 4, 0, 4]
        figure : matplotlib.figure.Figure, optional
            Figure to use for rendering. If None, creates a new figure.
        """
        self.L = L
        
        if axis_limits is None:
            axis_limits = [-4, 4, -4, 4, 0, 4]
        self.axis_limits = axis_limits
        
        # Create or use provided figure
        if figure is None:
            self.fig = plt.figure(figsize=(10, 8))
        else:
            self.fig = figure
        
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Storage for rendered objects
        self.link_lines = []
        self.joint_markers = []
        self.frame_lines = []
        self.trajectory_plot = None
        self.trajectory_points = []
        
        self._setup_axes()
    
    def _setup_axes(self):
        """Set up 3D axes with appropriate limits and labels."""
        self.ax.set_xlim(self.axis_limits[0], self.axis_limits[1])
        self.ax.set_ylim(self.axis_limits[2], self.axis_limits[3])
        self.ax.set_zlim(self.axis_limits[4], self.axis_limits[5])
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.set_box_aspect([1, 1, 0.67])
        
        # Set view angle for optimal visualization
        self.ax.view_init(elev=35.264, azim=45)
    
    def _create_coordinate_frame(self, T, length=1.0):
        """
        Create coordinate frame axes at a given transformation.
        
        Parameters
        ----------
        T : ndarray (4, 4)
            Transformation matrix
        length : float
            Length of frame axes
            
        Returns
        -------
        lines : list
            List of line objects [x_axis, y_axis, z_axis]
        """
        origin = T[:3, 3]
        
        # Axis directions
        x_dir = T[:3, 0] * length
        y_dir = T[:3, 1] * length
        z_dir = T[:3, 2] * length
        
        # Create lines
        x_line = self.ax.plot([origin[0], origin[0] + x_dir[0]],
                              [origin[1], origin[1] + x_dir[1]],
                              [origin[2], origin[2] + x_dir[2]],
                              'k-', linewidth=2)[0]  # Black for X
        
        y_line = self.ax.plot([origin[0], origin[0] + y_dir[0]],
                              [origin[1], origin[1] + y_dir[1]],
                              [origin[2], origin[2] + y_dir[2]],
                              'k-', linewidth=2)[0]  # Black for Y
        
        z_line = self.ax.plot([origin[0], origin[0] + z_dir[0]],
                              [origin[1], origin[1] + z_dir[1]],
                              [origin[2], origin[2] + z_dir[2]],
                              'r-', linewidth=2)[0]  # Red for Z
        
        return [x_line, y_line, z_line]
    
    def update(self, q, draw_trajectory=False):
        """
        Update the robot visualization for a given configuration.
        
        Parameters
        ----------
        q : array_like (3,)
            Joint angles [q1, q2, q3]
        draw_trajectory : bool, default=False
            If True, draws the end-effector trajectory
            
        Returns
        -------
        artists : list
            List of updated artist objects
        """
        q = np.asarray(q)
        
        # Compute forward kinematics
        T10, T20, T30 = forward_kinematics(q, self.L)
        
        # Clear previous link lines and frames
        for line in self.link_lines:
            line.remove()
        self.link_lines = []
        
        for marker in self.joint_markers:
            marker.remove()
        self.joint_markers = []
        
        for lines in self.frame_lines:
            for line in lines:
                line.remove()
        self.frame_lines = []
        
        # Extract joint positions
        origin = np.array([0, 0, 0])
        p1 = T10[:3, 3]
        p2 = T20[:3, 3]
        p3 = T30[:3, 3]
        
        # Draw links as lines
        link1 = self.ax.plot([origin[0], p1[0]], [origin[1], p1[1]], [origin[2], p1[2]], 
                             'g-', linewidth=4)[0]
        link2 = self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                             'm-', linewidth=3)[0]
        link3 = self.ax.plot([p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]], 
                             'y-', linewidth=2)[0]
        
        self.link_lines = [link1, link2, link3]
        
        # Draw joints as markers
        joint0 = self.ax.plot([origin[0]], [origin[1]], [origin[2]], 
                              'ko', markersize=8)[0]
        joint1 = self.ax.plot([p1[0]], [p1[1]], [p1[2]], 
                              'go', markersize=6)[0]
        joint2 = self.ax.plot([p2[0]], [p2[1]], [p2[2]], 
                              'mo', markersize=6)[0]
        joint3 = self.ax.plot([p3[0]], [p3[1]], [p3[2]], 
                              'yo', markersize=6)[0]
        
        self.joint_markers = [joint0, joint1, joint2, joint3]
        
        # Add coordinate frames
        transforms = [T10, T20, T30]
        for T in transforms:
            frame_lines = self._create_coordinate_frame(T, length=0.5)
            self.frame_lines.append(frame_lines)
        
        # Draw trajectory if requested
        if draw_trajectory:
            end_effector_pos = T30[:3, 3]
            self.trajectory_points.append(end_effector_pos.copy())
            
            if len(self.trajectory_points) > 1:
                traj = np.array(self.trajectory_points)
                if self.trajectory_plot is not None:
                    self.trajectory_plot.remove()
                self.trajectory_plot = self.ax.plot(
                    traj[:, 0], traj[:, 1], traj[:, 2],
                    'c-', linewidth=1, alpha=0.5
                )[0]
        
        return self.link_lines + self.joint_markers + [line for lines in self.frame_lines for line in lines]
    
    def clear_trajectory(self):
        """Clear the trajectory history."""
        self.trajectory_points = []
        if self.trajectory_plot is not None:
            self.trajectory_plot.remove()
            self.trajectory_plot = None
    
    def animate(self, q_trajectory, dt=0.05, draw_trajectory=True):
        """
        Animate the robot through a trajectory.
        
        Parameters
        ----------
        q_trajectory : ndarray (n, 3)
            Array of joint configurations, one per time step
        dt : float, default=0.05
            Time delay between frames [seconds]
        draw_trajectory : bool, default=True
            If True, draws the end-effector trajectory
        """
        plt.ion()  # Interactive mode
        self.clear_trajectory()
        
        for i, q in enumerate(q_trajectory):
            self.update(q, draw_trajectory=draw_trajectory)
            plt.pause(dt)
            
            if i == 0:
                # Pause at the start
                plt.pause(0.5)
        
        plt.ioff()
        plt.show()
