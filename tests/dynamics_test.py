import numpy as np
from scipy.integrate import solve_ivp
from vtvl_sim.dynamics import lander_eom
from vtvl_sim.params import PARAMS


def test_free_fall():
    """Zero thrust should reduce exactly to projectile motion.

    T=0 => mdot=0 (no propellant burned coasting), so mass along for the ride
    at index 6 stays exactly at its initial value — checked as a bonus.
    """
    T, delta = 0.0, 0.0
    x0, z0 = 0.0, 100.0
    xdot0, zdot0 = 5.0, 0.0
    m0 = 200.0
    state0 = [x0, z0, xdot0, zdot0, 0.0, 0.0, m0]

    t_span = (0, 4.0)
    t_eval = np.linspace(*t_span, 200)

    sol = solve_ivp(
        lander_eom, t_span, state0,
        args=(T, delta, PARAMS),
        t_eval=t_eval, rtol=1e-9, atol=1e-9
    )

    t = sol.t
    x_an = x0 + xdot0 * t
    z_an = z0 + zdot0 * t - 0.5 * PARAMS['g'] * t**2

    assert np.max(np.abs(sol.y[0] - x_an)) < 1e-6
    assert np.max(np.abs(sol.y[1] - z_an)) < 1e-6
    assert np.max(np.abs(sol.y[6] - m0)) < 1e-9


def test_hover_equilibrium():
    """Continuously re-deriving T = m(t)*g should hold every state but mass
    constant, even as mass depletes.

    A *fixed* T=m0*g (correct at t=0) does not hold hover once mass is a real
    state: weight drops as propellant burns while thrust stays put, so the
    vehicle drifts upward. Re-deriving T from the instantaneous mass each RHS
    call — exactly what the closed-loop controllers' feedforward already does
    (PLAN.md §2.4) — keeps thrust exactly matching weight at every instant, so
    x, z, xdot, zdot, theta, thetadot are analytically constant for all time
    regardless of how much mass has burned off; only mass itself moves.
    """
    g = PARAMS['g']
    state0 = [0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 200.0]

    def rhs(t, state):
        m = state[6]
        return lander_eom(t, state, m * g, 0.0, PARAMS)

    sol = solve_ivp(
        rhs, (0, 10.0), state0,
        t_eval=np.linspace(0, 10.0, 200),
        rtol=1e-9, atol=1e-9
    )

    drift = np.abs(sol.y[:6, -1] - np.array(state0[:6]))
    assert np.max(drift) < 1e-4
    assert sol.y[6, -1] < state0[6]  # mass did actually deplete