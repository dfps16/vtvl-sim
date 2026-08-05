"""Reference physical parameters and baseline gains for the notebooks and tests.

Runtime configuration is JSON-driven: the simulator (via run_scenarios, or any
downstream consumer calling build_setup) reads scenario files validated by
schemas.py, so *those* are the source of truth for a run. This module is the shared default set the diagnostic notebooks and the
dynamics tests import, kept numerically consistent with test_scenarios/*.json.
"""

import numpy as np

# Max thrust sets the scale; T_min is a fixed fraction of it (the non-zero
# lower bound is the non-convex constraint that motivates G-FOLD later).
T_MAX = 2500.0  # N, ~1.27x hover weight (m*g = 1962 N at m=200 kg)

PARAMS = {
    'm': 200.0,                   # kg   dry mass
    'I': 200.0,                   # kg.m^2  pitch moment of inertia
    'L': 0.5,                     # m    CoM-to-gimbal moment arm
    'g': 9.81,                    # m/s^2
    'T_min': 0.4 * T_MAX,         # N    min throttle
    'T_max': T_MAX,               # N    max thrust
    'isp': 200,
    'delta_max': np.radians(12),
    'tilt_limit': np.radians(10),  # rad  gimbal deflection limit
}

# Baseline altitude-PID gains (Week 1).
PID_GAINS = {
    'kp': 3.0,
    'ki': 0.0,
    'kd': 30.0,
}

# Inner attitude-loop PD gains (Week 2).
PD_GAINS = {
    'kp': 16.0,
    'kd': 6.4,
}
# Use the above PD or the below zeta and omega_n values
SYS_PROP_ATT = { # Target system properties for the attitude loop
    'zeta': 0.8, 
    'omega_n': 4, # rad/s
}

# Full cascade design properties (PD gains are worked out from this)
SYS_PROP_CASC = {
    'zeta_x': 1.0, 
    'omega_x': 0.4, # rad/s
    'zeta_z': 1.0, 
    'omega_z': 0.4, # rad/s
    'zeta_theta': 0.8, 
    'omega_theta': 4, # rad/s
}