
import numpy as np
from scipy.integrate import cumulative_trapezoid

from vtvl_sim.paths import result_path


def save_csv(sim_results, save_path=result_path('last_sim_data.csv')):
    import csv
    t = sim_results['t']
    x = sim_results['x']
    z = sim_results['z']
    xdot = sim_results['xdot']
    zdot = sim_results['zdot']
    theta = sim_results['theta']
    thetadot = sim_results['thetadot']

    with open(save_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'z', 'xdot', 'zdot', 'theta', 'thetadot'])
        for row in zip(t, x, z, xdot, zdot, theta, thetadot):
            writer.writerow(row)

def engine_signals(sim_results, sim_setup):
    """Thrust and throttle, commanded (pre-saturation) and actual (applied).

    Single definition of 'throttle' — a fraction of T_max — shared by the engine
    plot, the animation HUD and the engine metrics.
    """
    T_max = sim_setup['params']['T_max']
    T_cmd = sim_results['T_cmd']   # raw demand m·z̈_des, before the [T_min, T_max] clip
    T_act = sim_results['u_T']     # post-clip; with no engine lag this is what lander_eom applied
    return T_cmd, T_act, T_cmd / T_max, T_act / T_max


def compute_state_metrics(sim_results):
    """Peak excursions and the terminal state vector, for the State tab.

    Angles are returned in degrees to match the State plot they sit under; the
    underlying arrays are radians.
    """
    xdot = sim_results['xdot']
    zdot = sim_results['zdot']
    theta = sim_results['theta']
    thetadot = sim_results['thetadot']

    return {
        'max_speed': np.max(np.sqrt(xdot ** 2 + zdot ** 2)),
        'max_rate_deg': np.max(np.abs(np.degrees(thetadot))),
        'max_attitude_deg': np.max(np.abs(np.degrees(theta))),
        'final_state': {
            'x': sim_results['x'][-1],
            'z': sim_results['z'][-1],
            'xdot': xdot[-1],
            'zdot': zdot[-1],
            'theta': np.degrees(theta[-1]),
            'thetadot': np.degrees(thetadot[-1]),
        },
    }


def compute_trajectory_metrics(sim_setup, sim_results):
    """Touchdown metrics plus apogee and the ascent/descent propellant split."""
    metrics = compute_touchdown_metrics(sim_setup, sim_results)

    params = sim_setup['params']
    t = sim_results['t']
    z = sim_results['z']
    thrust = sim_results['u_T']

    # Constant-mass model, so propellant is only ever a post-hoc integral of the
    # thrust. Split it at apogee: integrating the rate cumulatively and taking a
    # difference keeps ascent + descent == the total exactly, which masking the
    # trapezoid by a boolean would not (it breaks contiguity at the cut).
    i_apogee = int(np.argmax(z))
    mdot = thrust / (params['g'] * params['isp'])
    cum_propellant = cumulative_trapezoid(mdot, t, initial=0.0)

    metrics['apogee'] = np.max(z)
    metrics['propellant_ascent'] = cum_propellant[i_apogee]
    metrics['propellant_descent'] = cum_propellant[-1] - cum_propellant[i_apogee]
    return metrics


def compute_engine_metrics(sim_setup, sim_results):
    """Throttle envelope, in percent of T_max, for the Engine tab.

    Throttle is the *applied* thrust, so the minimum pins at T_min/T_max (40% at
    the default vehicle) whenever the engine deep-throttles — that is the throttle
    the engine delivered, not a stuck value. The average is time-weighted because
    solve_ivp's steps are non-uniform: a plain mean over-counts the transients,
    where the integrator clusters its samples.
    """
    t = sim_results['t']
    _, _, _, throttle = engine_signals(sim_results, sim_setup)

    return {
        'throttle_max': 100.0 * np.max(throttle),
        'throttle_min': 100.0 * np.min(throttle),
        'throttle_avg': 100.0 * np.trapezoid(throttle, t) / (t[-1] - t[0]),
    }


def compute_touchdown_metrics(sim_setup, sim_results):
    """Touchdown/usage metrics shared by write_sim_report and the GUI summary."""
    phases = sim_setup['phases']
    x_target, z_target, *_ = phases[-1]
    params = sim_setup['params']

    thrust = sim_results['u_T']
    isp = params['isp']

    t = sim_results['t']
    x = sim_results['x']
    z = sim_results['z']
    xdot = sim_results['xdot']
    zdot = sim_results['zdot']
    theta = sim_results['theta']
    m = sim_results['m']

    vel_touchdown = np.sqrt(xdot[-1] ** 2 + zdot[-1] ** 2)
    angle_touchdown_deg = np.degrees(theta[-1])

    altitude_error = z_target - z[-1]
    touchdown_error = x_target - x[-1]
    touchdown_time = t[-1]
    # Symmetric: only within landing_tolerance of the target counts as landed.
    # The old one-sided check (altitude_error < tolerance) was true for *any*
    # altitude at or above the target, so a flameout stopped mid-air also read
    # as a touchdown. The +1e-6 slack absorbs solve_ivp's event root-finding
    # residual: touchdown_event stops exactly at z == landing_tolerance, so a
    # genuine touchdown lands right on the boundary, not strictly inside it.
    landed = abs(altitude_error) <= sim_setup['landing_tolerance'] + 1e-6
    propellant_remaining = m[-1] - params['m_dry']
    flameout = (not landed) and propellant_remaining <= 1e-6

    total_impulse = np.trapezoid(thrust, t)
    # Post-hoc estimate: integrating T/(g*isp) over the recorded thrust trace.
    propellant_mass = total_impulse / (params['g'] * isp)
    # Actual: the real mass loss the ODE integrator produced. These two are the
    # same physical quantity computed two different ways (Tsiolkovsky integral
    # vs. direct state integration) and should agree closely — see PLAN.md §2.7.
    propellant_actual = m[0] - m[-1]

    return {
        'landed': landed,
        'flameout': flameout,
        'touchdown_time': touchdown_time,
        'touchdown_error': touchdown_error,
        'vel_touchdown': vel_touchdown,
        'angle_touchdown_deg': angle_touchdown_deg,
        'altitude_error': altitude_error,
        'total_impulse': total_impulse,
        'propellant_mass': propellant_mass,
        'propellant_remaining': propellant_remaining,
        'propellant_actual': propellant_actual,
    }


def write_sim_report(sim_setup, sim_results, save_path=result_path('last_sim_report.txt')):
    m = compute_touchdown_metrics(sim_setup, sim_results)

    with open(save_path, 'w') as f:
        f.write("====== SIM REPORT ======\n")
        if m['landed']:
            f.write("Touchdown state --\n")
            f.write(f"Touchdown time {m['touchdown_time']:.3g} sec --\n")
            f.write(f"Touchdown x-error {m['touchdown_error']:.3g} m --\n")
            f.write(f"Touchdown velocity {m['vel_touchdown']:.3g} m/s --\n")
            f.write(f"Touchdown angle {m['angle_touchdown_deg']:.3g} deg --\n")
        elif m['flameout']:
            f.write("Flameout --\n")
            f.write(f"Final altitude: {m['altitude_error']:.3g} m --\n")
            f.write(f"Final x error: {m['touchdown_error']:.3g} m --\n")
            f.write(f"Propellant remaining {m['propellant_remaining']:.3g} kg --\n")
        else:
            f.write("No touchdown achieved --\n")
            f.write(f"Final altitude: {m['altitude_error']:.3g} m --\n")
            f.write(f"Final x error: {m['touchdown_error']:.3g} m --\n")
            f.write(f"Propellant remaining {m['propellant_remaining']:.3g} kg --\n")
        f.write("Usage --\n")
        f.write(f"Total impulse required {m['total_impulse']:.2f} Ns\n")
        f.write(f"Propellant mass (post-hoc estimate, from thrust integral) {m['propellant_mass']:.2f} kg\n")
        f.write(f"Propellant mass (actual, integrated by the solver) {m['propellant_actual']:.2f} kg\n")
        f.write(f"Cross-check difference {abs(m['propellant_mass'] - m['propellant_actual']):.4f} kg\n")