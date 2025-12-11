"""Control module for PD and inverse dynamics control."""

from .inverse_dynamics import pd_control, compute_control_torques

__all__ = ["pd_control", "compute_control_torques"]

