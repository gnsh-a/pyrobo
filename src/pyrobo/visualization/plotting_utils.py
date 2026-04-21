"""
Plotting utilities for trajectory, energy, and tracking analysis.
"""

import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory(t, x, y, z, title="End-Effector Trajectory", figsize=(10, 8)):
    """
    Plot end-effector position components vs time.
    
    Parameters
    ----------
    t : array_like
        Time vector
    x, y, z : array_like
        Position components
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
        
    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure object
    """
    fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
    
    axes[0].plot(t, x, 'r-', linewidth=1.5)
    axes[0].set_ylabel('x [m]')
    axes[0].grid(True)
    axes[0].set_title(title)
    
    axes[1].plot(t, y, 'g-', linewidth=1.5)
    axes[1].set_ylabel('y [m]')
    axes[1].grid(True)
    
    axes[2].plot(t, z, 'b-', linewidth=1.5)
    axes[2].set_ylabel('z [m]')
    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True)
    
    plt.tight_layout()
    return fig


def plot_energy(t, T, V, E=None, title="System Energy", figsize=(10, 6)):
    """
    Plot kinetic, potential, and total energy vs time.
    
    Parameters
    ----------
    t : array_like
        Time vector
    T : array_like
        Kinetic energy
    V : array_like
        Potential energy
    E : array_like, optional
        Total energy. If None, computed as T + V
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
        
    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure object
    """
    if E is None:
        E = np.asarray(T) + np.asarray(V)
    
    fig, ax = plt.subplots(figsize=figsize)
    
    ax.plot(t, E, 'b-', linewidth=2, label='Total Energy')
    ax.plot(t, V, 'g-', linewidth=1.5, label='Potential Energy')
    ax.plot(t, T, 'r-', linewidth=1.5, label='Kinetic Energy')
    
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Energy [J]')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    return fig


def plot_joint_tracking(t, q, q_des, joint_names=None, 
                       title="Joint Tracking", figsize=(10, 10)):
    """
    Plot joint positions vs desired positions.
    
    Parameters
    ----------
    t : array_like
        Time vector
    q : ndarray (n, 3)
        Actual joint positions
    q_des : ndarray (n, 3)
        Desired joint positions
    joint_names : list of str, optional
        Names for each joint
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
        
    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure object
    """
    if joint_names is None:
        joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    
    fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
    
    for i in range(3):
        axes[i].plot(t, q[:, i], 'b-', linewidth=2, label=f'Actual')
        axes[i].plot(t, q_des[:, i], 'r--', linewidth=1.5, label=f'Desired')
        axes[i].set_ylabel(f'{joint_names[i]} [rad]')
        axes[i].legend()
        axes[i].grid(True)
        
        if i == 0:
            axes[i].set_title(title)
    
    axes[2].set_xlabel('Time [s]')
    
    plt.tight_layout()
    return fig


def plot_joint_errors(t, q, q_des, joint_names=None,
                     title="Joint Tracking Errors", figsize=(10, 10)):
    """
    Plot tracking errors for each joint on separate subplots.
    
    Parameters
    ----------
    t : array_like
        Time vector
    q : ndarray (n, 3)
        Actual joint positions
    q_des : ndarray (n, 3)
        Desired joint positions
    joint_names : list of str, optional
        Names for each joint
    title : str, optional
        Overall title
    figsize : tuple, optional
        Figure size
        
    Returns
    -------
    figs : list of matplotlib.figure.Figure
        List of figure objects (one per joint)
    """
    if joint_names is None:
        joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    
    q = np.asarray(q)
    q_des = np.asarray(q_des)
    errors = q_des - q
    
    figs = []
    
    for i in range(3):
        fig, ax = plt.subplots(figsize=(10, 5))
        
        # Plot position and error on same axes
        ax.plot(t, q[:, i], 'b-', linewidth=2, label=f'{joint_names[i]}')
        ax.plot(t, errors[:, i], 'r--', linewidth=1.5, label=f'Error Δq{i+1}')
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Angle [rad]')
        ax.set_title(f'{title} - {joint_names[i]}')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        figs.append(fig)
    
    return figs


def plot_joint_velocities(t, qd, qd_des=None, joint_names=None,
                         title="Joint Velocities", figsize=(10, 10)):
    """
    Plot joint velocities vs time.
    
    Parameters
    ----------
    t : array_like
        Time vector
    qd : ndarray (n, 3)
        Actual joint velocities
    qd_des : ndarray (n, 3), optional
        Desired joint velocities
    joint_names : list of str, optional
        Names for each joint
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
        
    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure object
    """
    if joint_names is None:
        joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    
    fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)
    
    for i in range(3):
        axes[i].plot(t, qd[:, i], 'b-', linewidth=2, label='Actual')
        if qd_des is not None:
            axes[i].plot(t, qd_des[:, i], 'r--', linewidth=1.5, label='Desired')
            axes[i].legend()
        axes[i].set_ylabel(f'{joint_names[i]} [rad/s]')
        axes[i].grid(True)
        
        if i == 0:
            axes[i].set_title(title)
    
    axes[2].set_xlabel('Time [s]')
    
    plt.tight_layout()
    return fig


def plot_spline_trajectory(t, q, dq, title="Cubic Spline Trajectory", figsize=(10, 8)):
    """
    Plot position and velocity for a cubic spline trajectory.
    
    Parameters
    ----------
    t : array_like
        Time vector
    q : array_like
        Position trajectory
    dq : array_like
        Velocity trajectory
    title : str, optional
        Plot title
    figsize : tuple, optional
        Figure size
        
    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure object
    """
    fig, axes = plt.subplots(2, 1, figsize=figsize, sharex=True)
    
    axes[0].plot(t, q, 'b-', linewidth=2)
    axes[0].set_ylabel('Position [rad]')
    axes[0].set_title(title)
    axes[0].grid(True)
    
    axes[1].plot(t, dq, 'r-', linewidth=2)
    axes[1].set_ylabel('Velocity [rad/s]')
    axes[1].set_xlabel('Time [s]')
    axes[1].grid(True)
    
    plt.tight_layout()
    return fig

