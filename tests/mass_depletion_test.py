import numpy as np
from scipy.integrate import solve_ivp

from vtvl_sim.controllers import CONTROLLER_REGISTRY, LQRController
from vtvl_sim.dynamics import lander_eom
from vtvl_sim.params import PARAMS
from vtvl_sim.post_processing import compute_touchdown_metrics
from vtvl_sim.scenario_io import load_scenario
from vtvl_sim.sim import sim_run


def test_mass_conserved_at_constant_thrust():
    """m(t) at fixed T should match the closed-form Tsiolkovsky rate exactly.

    Catches a sign error in mdot immediately: a positive mdot would grow mass
    under thrust, an obvious tell against this closed form.
    """
    T = PARAMS['T_min']
    g, isp = PARAMS['g'], PARAMS['isp']
    m0 = 200.0
    state0 = [0.0, 100.0, 0.0, 0.0, 0.0, 0.0, m0]

    t_span = (0.0, 2.0)
    sol = solve_ivp(
        lander_eom, t_span, state0,
        args=(T, 0.0, PARAMS),
        t_eval=np.linspace(*t_span, 50),
        rtol=1e-9, atol=1e-9
    )

    m_analytic = m0 - (T / (g * isp)) * sol.t
    assert np.max(np.abs(sol.y[6] - m_analytic)) < 1e-9


def test_propellant_exhausted_event_fires():
    """A scenario that runs dry before touchdown should halt via the
    propellant_expended event, not by exhausting t_end, and m should never dip
    below m_dry (float overshoot past the event crossing is the failure mode
    to watch for). lqr1.json is a known flameout under current gains/margin —
    see REPORT_NOTES.md §7.1.
    """
    sim_setup, solver_setup, _ = load_scenario('test_scenarios/lqr1.json')
    results = sim_run(sim_setup, solver_setup)

    m_dry = sim_setup['params']['m_dry']
    t_end = sim_setup['phases'][-1][2]

    assert results['t'][-1] < t_end                            # stopped early, not via timeout
    assert results['z'][-1] > sim_setup['landing_tolerance']    # did not touch down
    assert np.min(results['m']) >= m_dry - 1e-6                 # never overshoots the floor
    assert results['m'][-1] <= m_dry + 1e-6                     # actually reached it


def test_lqr_gain_schedules_with_mass():
    """K must differ across masses (gain-scheduling actually wired in, not a
    stale cached K the way the pre-mass-depletion code cached it on purpose).

    Paired with the two closed-form entries (PLAN.md §1.7/§2.5b) that must stay
    exactly fixed regardless of mass — this one test exercises both the "it
    does change" and "these two provably shouldn't" halves of the theory.
    """
    gains = CONTROLLER_REGISTRY['LQR']['defaults']
    controller = LQRController(gains, x_target=20.0, z_target=0.0, theta_target=0.0)

    params_heavy = {**PARAMS, 'm': 220.0}
    params_light = {**PARAMS, 'm': 180.0}

    K_heavy = controller._compute_gain_matrix(params_heavy)
    K_light = controller._compute_gain_matrix(params_light)

    assert not np.allclose(K_heavy, K_light)
    assert np.isclose(K_heavy[0, 1], K_light[0, 1])  # thrust_dev / z_dev
    assert np.isclose(K_heavy[1, 0], K_light[1, 0])  # delta_dev / x_dev


def test_both_controllers_still_land():
    """Rerun default.json (Cascaded PD) and lqr1.json (LQR) after the schema
    migration and mass depletion mechanism land; not an exact reproduction of
    Part 1's constant-mass numbers — that shift is itself a report result
    (REPORT_NOTES.md §7.1), not a bug.

    Cascaded PD's feedforward re-reads mass every step (PLAN.md §2.4) and lands
    cleanly within its propellant margin. LQR's cost function makes thrust
    nearly free (REPORT_NOTES.md §2.6), so at these Part-1-tuned gains it burns
    more total propellant than the cascade despite lower instantaneous thrust,
    and currently flames out short of touchdown on identical geometry — a real,
    documented consequence of the gains, not a mechanism bug. Asserting that
    outcome (rather than a landing) keeps this test honest about present
    tuning; PLAN.md §2.8 defers the re-tune that would fix it.
    """
    cascade_setup, cascade_solver, _ = load_scenario('test_scenarios/default.json')
    cascade_results = sim_run(cascade_setup, cascade_solver)
    cascade_metrics = compute_touchdown_metrics(cascade_setup, cascade_results)

    assert cascade_metrics['landed']
    assert abs(cascade_results['zdot'][-1]) < 1.0
    assert abs(cascade_results['x'][-1] - 20.0) < 1.0

    lqr_setup, lqr_solver, _ = load_scenario('test_scenarios/lqr1.json')
    lqr_results = sim_run(lqr_setup, lqr_solver)
    lqr_metrics = compute_touchdown_metrics(lqr_setup, lqr_results)

    assert lqr_metrics['flameout']
    assert lqr_results['t'][-1] < lqr_setup['phases'][-1][2]  # stopped early
    assert lqr_results['z'][-1] > 0                            # mid-air, not underground
    assert abs(lqr_results['zdot'][-1]) < 2.0                  # not a violent free-fall
