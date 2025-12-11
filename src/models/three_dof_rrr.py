"""
Model parameters for the 3-DOF RRR manipulator.
"""


def initialize_three_dof_rrr():
    """
    Initialize model parameters for the 3-DOF RRR manipulator.
    
    Returns
    -------
    model_params : dict
        Dictionary containing:
        - g : float
            Gravitational constant [m/s^2]
        - m : float
            Link mass [kg]
        - L : float
            Link length [m]
        - I : float
            Link inertia (perpendicular to link's centerline) [kg*m^2]
    """
    model_params = {
        'g': 9.81,      # gravitational constant [m/s^2]
        'm': 10.0,      # link mass [kg]
        'L': 2.0,       # link length [m]
        'I': 5.0,       # link inertia [kg*m^2]
    }
    
    return model_params

