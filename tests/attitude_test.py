"""Inner attitude-loop regression suite — see tests/attitude_test_plan.md.

Validates AttitudePDController (gimbal delta -> pitch theta, PD + dynamic
inversion) against its linear design targets SYS_PROP_ATT (zeta, omega_n).
dynamics_test.py covers the open-loop plant; this covers the closed inner loop.

Scope: isolated inner loop at fixed throttle (T = m*g, held at the reference
mass in params.py — thetaddot has no m dependence at all, so this is exact
regardless of what mass depletion is doing elsewhere; only the vertical/
lateral channels care about mass). Translational coupling, disturbance
rejection, and large-angle behaviour beyond saturation are out of scope.
"""

import numpy as np
from scipy.integrate import solve_ivp

from vtvl_sim.controllers import AttitudePDController
from vtvl_sim.dynamics import lander_eom
from vtvl_sim.params import PARAMS, SYS_PROP_ATT

_MAX_STEP = 0.005


def _closed_loop_rhs(t, state, controller, params):
    thrust, delta = controller(t, state, params)  # already actuator-clipped by the controller
    return lander_eom(t, state, thrust, delta, params)


def _pd_gains():
    """kp = omega_n^2, kd = 2*zeta*omega_n — derived from SYS_PROP_ATT so this
    test tracks any retune automatically rather than duplicating hardcoded
    numbers."""
    zeta = SYS_PROP_ATT['zeta']
    omega_n = SYS_PROP_ATT['omega_n']
    return {'kp': omega_n ** 2, 'kd': 2 * zeta * omega_n}, zeta, omega_n


def _run_pitch_step(theta_cmd_deg, t_end=3.0):
    gains, zeta, omega_n = _pd_gains()
    theta_cmd = np.radians(theta_cmd_deg)
    controller = AttitudePDController(gains, theta_cmd)

    # Level, at rest, floating at hover. The mass entry is a placeholder only
    # dynamics.py's state-unpacking needs it (index 6); the controller reads
    # the reference mass from PARAMS, not from this state, so its drift here
    # (governed by whatever thrust the controller happens to command) is
    # irrelevant to theta, thetadot, or delta.
    state0 = [0.0, 100.0, 0.0, 0.0, 0.0, 0.0, PARAMS['m']]

    sol = solve_ivp(
        _closed_loop_rhs, (0.0, t_end), state0,
        args=(controller, PARAMS),
        max_step=_MAX_STEP, rtol=1e-9, atol=1e-9,
    )

    # delta is recomputed post-hoc from the solution states — exact, since the
    # controller is stateless (rate measured, no integrator).
    delta = np.array([controller(ti, sol.y[:, i], PARAMS)[1]
                       for i, ti in enumerate(sol.t)])

    return sol.t, sol.y[4], delta, theta_cmd, zeta, omega_n


def test_steady_state_tracks_command():
    """Final theta should reach theta_cmd with no steady-state offset."""
    t, theta, delta, theta_cmd, *_ = _run_pitch_step(theta_cmd_deg=2.0, t_end=3.0)
    assert abs(theta[-1] - theta_cmd) < np.radians(0.05)


def test_overshoot_matches_linear_prediction():
    """Peak overshoot should match the standard 2nd-order prediction
    exp(-zeta*pi / sqrt(1-zeta^2)) — the strongest check here, since it
    confirms the closed loop sits at the *designed* pole locations rather than
    merely being stable.
    """
    t, theta, delta, theta_cmd, zeta, omega_n = _run_pitch_step(theta_cmd_deg=2.0, t_end=3.0)

    predicted_pct = 100.0 * np.exp(-zeta * np.pi / np.sqrt(1 - zeta ** 2))
    observed_pct = 100.0 * (np.max(theta) - theta_cmd) / theta_cmd

    assert abs(observed_pct - predicted_pct) < 0.5


def test_sign_convention_first_delta_negative_for_positive_step():
    """A positive theta_cmd should command a negative delta first — guards
    against a flipped sign in the EOM torque or the controller's inversion."""
    t, theta, delta, theta_cmd, *_ = _run_pitch_step(theta_cmd_deg=2.0, t_end=3.0)
    assert delta[0] < 0.0


def test_gimbal_saturates_and_respects_limit():
    """A large step should drive delta into saturation, and it should never
    exceed +-delta_max regardless of how large the demand gets."""
    t, theta, delta, theta_cmd, *_ = _run_pitch_step(theta_cmd_deg=8.0, t_end=3.0)
    delta_max = PARAMS['delta_max']

    assert np.max(np.abs(delta)) <= delta_max + 1e-9
    assert abs(np.max(np.abs(delta)) - delta_max) < 1e-3
