import numpy as np


def lander_eom(t, state, thrust, delta, params):
    """Equations of motion for a 2D VTVL lander (planar, rigid body).

    Thrust T acts at the base, gimbal angle delta rotates it relative to body axis.
    State: [x, z, xdot, zdot, theta, thetadot] — position, velocity, attitude, rate.
    Returns state derivatives for use with an ODE integrator (e.g. scipy solve_ivp).
    """
    x, z, xdot, zdot, theta, thetadot, m = state
    I, L, g, isp = params['I'], params['L'], params['g'], params['isp']

    # Inertial frame accelerations from thrust projected through body + gimbal angles
    xddot = - (thrust * np.sin(theta - delta)) / m
    zddot = (thrust * np.cos(theta - delta) - m * g) / m
    # Torque from off-axis thrust: moment arm L from CoM to nozzle
    thetaddot = - thrust * np.sin(delta) * L / I
    # Mass depletion from rocket engine
    m_dot = - thrust / (g * isp)
    return [xdot, zdot, xddot, zddot, thetadot, thetaddot, m_dot]