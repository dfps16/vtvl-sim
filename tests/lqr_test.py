import control
import numpy as np

from vtvl_sim.controllers import CONTROLLER_REGISTRY, bryson_weights, linearize_hover
from vtvl_sim.params import PARAMS
from vtvl_sim.scenario_io import load_scenario
from vtvl_sim.sim import sim_run


def test_lqr_controllability():
    A, B = linearize_hover(PARAMS)
    assert np.linalg.matrix_rank(control.ctrb(A, B)) == 6


def test_lqr_closed_loop_stable():
    A, B = linearize_hover(PARAMS)
    Q, R = bryson_weights(CONTROLLER_REGISTRY['LQR']['defaults'])
    K, *_ = control.lqr(A, B, Q, R)
    assert np.max(np.real(np.linalg.eigvals(A - B @ K))) < 0


def test_lqr_gain_structure():
    """Guards against a state-ordering transposition in bryson_weights.

    The structural-zero asserts alone do not catch this: permuting which
    deviation lands in which diagonal slot of Q leaves Q diagonal, so the
    block-sparse structure of K survives unchanged (REPORT_NOTES.md SS4.1).
    The closed-form asserts confirm each weight landed in the slot it was
    meant for, independent of that structural property.
    """
    A, B = linearize_hover(PARAMS)
    gains = CONTROLLER_REGISTRY['LQR']['defaults']
    Q, R = bryson_weights(gains)
    K, *_ = control.lqr(A, B, Q, R)

    assert np.allclose(K[0, [0, 2, 4, 5]], 0.0, atol=1e-9)
    assert np.allclose(K[1, [1, 3]], 0.0, atol=1e-9)

    assert np.isclose(K[0, 1], gains['thrust_dev'] / gains['z_dev'])
    assert np.isclose(K[1, 0], gains['delta_dev'] / gains['x_dev'])


def test_lqr_lands():
    """Regression guard against a crashing tune: the touchdown event alone
    (z <= landing_tolerance) does not check survivability, so the crashing
    handover-default gains (touchdown at -15.0 m/s) would also pass it.
    """
    sim_setup, solver_setup, _ = load_scenario('test_scenarios/lqr1.json')
    results = sim_run(sim_setup, solver_setup)

    assert results['t'][-1] < 30.0
    assert abs(results['zdot'][-1]) < 1.0
    assert abs(results['x'][-1] - 20.0) < 1.0
