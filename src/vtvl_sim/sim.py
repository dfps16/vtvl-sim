import numpy as np
from scipy.integrate import solve_ivp

from vtvl_sim.controllers import _RECORD_KEYS
from vtvl_sim.dynamics import lander_eom


def closed_loop_rhs(t, state, sim_setup, controller):
    """RHS passed to solve_ivp: query controller, saturate actuators, return state derivatives."""
    params = sim_setup['params']
    ctrl_params = {**params, 'm': state[6]}
    thrust, delta = controller(t, state, ctrl_params)
    # Clamp magnitude, then clamp gimbal angle
    thrust = np.clip(thrust, params['T_min'], params['T_max'])
    delta = np.clip(delta, -params['delta_max'], params['delta_max'])
    state_dot = lander_eom(t, state, thrust, delta, params)
    return state_dot

def touchdown_event(t, state, sim_setup, controller):
    z = state[1] # integration stops when z crosses zero from above
    touchdown = z - sim_setup['landing_tolerance']
    return touchdown

def propellant_expended_event(t, state, sim_setup, controller):
    return state[6] - sim_setup['params']['m_dry']

def vtvl_solver(state_0, x_target, z_target, theta_target, t_end, sim_setup, solver_setup):

    # Extracting the simulation setup
    PARAMS = sim_setup['params']
    GAINS = sim_setup['gains']
    SELECTED_CONTROLLER = sim_setup['controller']

    # Extracting the solver setup
    max_step = solver_setup['max_step']
    method = solver_setup['method']

    controller = SELECTED_CONTROLLER['build'](GAINS, x_target, z_target, theta_target)

    # Setting up termination event(s)
    # Terminal + direction=-1: stop only when z is decreasing through zero (descending touchdown)
    touchdown_event.terminal = True
    touchdown_event.direction = -1

    propellant_expended_event.terminal = True
    propellant_expended_event.direction = -1
    
    sol = solve_ivp(
        closed_loop_rhs,
        (0.0, t_end),
        state_0,
        args=(sim_setup, controller),  # must be a tuple matching (params, controller) after (t, state)
        events=(touchdown_event, propellant_expended_event),
        max_step=max_step,
        method=method
    )

    t = sol.t
    x = sol.y[0]
    z = sol.y[1]
    xdot = sol.y[2]
    zdot = sol.y[3]
    theta = sol.y[4]
    thetadot = sol.y[5]
    m = sol.y[6]

    # Recover the per-step control diagnostics (applied thrust/gimbal and the
    # pre-saturation demands) by replaying each controller's stateless record()
    # over the solution states. Any controller in the registry can be recorded
    # this way — the solver no longer depends on the cascade's internals. Signals
    # a given controller does not produce come back as NaN.
    recorded = {k: np.empty(t.size) for k in _RECORD_KEYS}
    for i in range(t.size):
        ctrl_params_i = {**PARAMS, 'm': sol.y[6, i]}
        rec = controller.record(sol.y[:, i], ctrl_params_i)
        for k in _RECORD_KEYS:
            recorded[k][i] = rec[k]

    results = {
        't': t,
        'x': x,
        'z': z,
        'xdot': xdot,
        'zdot': zdot,
        'theta': theta,
        'thetadot': thetadot,
        'm': m,
        **recorded,
    }

    return results

def sim_run(sim_setup, solver_setup):
    segments = []  # collects results after each phase
    state_n = list(sim_setup['initial_state'])  # state after each phase
    time_elapsed = 0.0  # time elapsed since start of sim

    # Loop through each phase, solve the dynamics, append the results
    for x_target, z_target, t_end, theta_target in sim_setup['phases']:
        seg = vtvl_solver(state_n, x_target, z_target, theta_target, t_end, sim_setup, solver_setup)
        seg['t'] = seg['t'] + time_elapsed
        segments.append(seg)
        state_n = [seg[k][-1] for k in ('x', 'z', 'xdot', 'zdot', 'theta', 'thetadot', 'm')]
        time_elapsed = seg['t'][-1]

    return {
        key: np.concatenate([segments[0][key], *(s[key][1:] for s in segments[1:])])
        
        for key in segments[0]
        }