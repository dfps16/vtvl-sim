import control as control
import numpy as np

# Canonical set of per-step diagnostic signals every controller reports through
# record(). The solver assembles these into arrays aligned with the solution
# time base; plotting/post-processing consume u_T, delta, T_cmd, delta_cmd. The
# *_des demands and theta_cmd are kept for controllers that produce them (the
# cascade) and left NaN otherwise — nothing downstream requires them yet.
_RECORD_KEYS = (
    'theta_cmd', 'u_T', 'delta', 'T_cmd', 'delta_cmd',
    'xddot_des', 'zddot_des', 'thetaddot_des',
)


def _blank_record():
    """A record dict with every diagnostic signal set to NaN, to be overwritten."""
    return {k: np.nan for k in _RECORD_KEYS}


def linearize_hover(params):
    """Jacobian linearisation of the lander dynamics about the hover trim point.

    Assumptions: constant mass, no aerodynamic forces, planar 3-DoF, rigid body motion only
    small angles about trim (theta = delta = 0), T = mg

    :param params: physical parameter dict; reads 'm', 'I', 'L', 'g' only. The trim
        thrust is derived as T = m*g, not read from the throttle bounds.
    :return: (A, B) for the linear model e_dot = A e + B u, where the state error is
        ordered [x, z, xdot, zdot, theta, thetadot] (matching dynamics.py) and the
        control is the *deviation* from trim, u = [T - m*g, delta]. Any weighting
        matrices built elsewhere must use this same ordering.
    """
    g = params['g']
    m = params['m']
    L = params['L']
    I = params['I']

    A = np.zeros((6, 6))
    B = np.zeros((6, 2))  # Initialising A and B matrices

    A[0, 2] = 1  # xdot column, x row
    A[1, 3] = 1  # zdot column, z row
    A[2, 4] = - g
    A[4, 5] = 1

    B[2, 1] = g
    B[3, 0] = 1 / m
    B[5, 1] = - m * g * L / I

    return A, B


def bryson_weights(targets):
    """LQR cost weights from Bryson's rule.

    Each variable is weighted by 1 / (acceptable deviation)^2, so every term in the
    cost is dimensionless and equals 1 when that variable sits exactly at its
    acceptable deviation.

    :param targets: the 8 acceptable deviations, keyed by gain field name. Angles
        ('theta_dev', 'thetadot_dev', 'delta_dev') are in RADIANS.
    :return Q, R: diagonal weight matrices. Q is ordered
        [x, z, xdot, zdot, theta, thetadot] and R is ordered [T, delta], matching the
        state and control ordering of linearize_hover's A and B.
    """
    # Initialising Q and R matrices
    Q = np.zeros((6, 6))
    R = np.zeros((2, 2))

    # Extracting target values from the target dictionary, and assigning to correct diagonal
    Q[0, 0] = 1 / targets['x_dev'] ** 2
    Q[1, 1] = 1 / targets['z_dev'] ** 2
    Q[2, 2] = 1 / targets['xdot_dev'] ** 2
    Q[3, 3] = 1 / targets['zdot_dev'] ** 2
    Q[4, 4] = 1 / targets['theta_dev'] ** 2
    Q[5, 5] = 1 / targets['thetadot_dev'] ** 2
    R[0, 0] = 1 / targets['thrust_dev'] ** 2
    R[1, 1] = 1 / targets['delta_dev'] ** 2

    return Q, R


class AltitudePIDController:
    """Altitude-only 1-DOF controller; gimbal fixed at δ=0, single actuator: thrust T.

    Baseline 0 — vertical hover/descent with mg feedforward, no lateral or
    attitude control. Suitable for a straight-down landing from a level state.
    """

    def __init__(self, gains, z_target):
        self.kp = gains['kp']
        self.ki = gains['ki']
        self.kd = gains['kd']
        self.r = z_target

        self.integral_sum = 0
        self.e_prev = None  # used by windup guard on the following call
        self.t_prev = None  # used to compute Δt on the following call

    def __call__(self, t, state, params):
        z = state[1]
        zdot = state[3]
        m = params['m']
        g = params['g']

        error = self.r - z

        # Skip derivative term on first call; Δt is undefined before t_prev is set
        if self.t_prev is not None:
            dt = t - self.t_prev
            d_term = zdot  # derivative-on-measurement: eliminates derivative kick on setpoint steps
        else:
            dt = 0
            d_term = 0

        # Conditional integration: integrator active only within |e| < 5 m to prevent windup
        # during large-error transients; resets to zero outside this band
        if self.e_prev is not None and np.abs(self.e_prev) < 5:
            self.integral_sum += error * dt
        else:
            self.integral_sum = 0

        self.e_prev = error
        self.t_prev = t

        # Gravity feedforward shifts the operating point to hover; PID corrects only residual deviations.
        # Eliminates steady-state thrust error without relying solely on the integrator.
        u_thrust = self.kp * error + self.ki * self.integral_sum - self.kd * d_term + m * g
        u_delta = 0.0  # gimbal fixed at zero; no lateral or attitude correction
        return (u_thrust, u_delta)

    def record(self, state, params):
        # Stateless thrust readout for diagnostics, aligned to the solution grid.
        # The integral term is replayed at its final value (self.integral_sum);
        # at the default ki=0 this is exact, and for ki≠0 the recorded thrust
        # demand omits transient integral action (a diagnostics-only caveat — the
        # integrated trajectory itself still reflects it).
        z = state[1]
        zdot = state[3]
        error = self.r - z
        thrust_cmd = self.kp * error + self.ki * self.integral_sum - self.kd * zdot \
                     + params['m'] * params['g']
        rec = _blank_record()
        rec.update({
            'theta_cmd': 0.0,
            'u_T': np.clip(thrust_cmd, params['T_min'], params['T_max']),
            'delta': 0.0,
            'T_cmd': thrust_cmd,
            'delta_cmd': 0.0,
        })
        return rec


class AttitudePDController:
    """Inner attitude loop as a standalone controller (hover-hold demo).

    Holds a fixed hover thrust T = m·g and drives pitch θ to a commanded
    reference θ_ref through PD-in-angular-acceleration + exact EOM inversion —
    the same law the cascade uses for its inner loop. There is no position or
    altitude control, so it does not land: it exercises the gimbal→pitch loop in
    isolation (mirrors notebooks/check_attitude.py and attitude_loop.py). θ_ref
    comes from the scenario's phase theta_target.
    """

    def __init__(self, gains, theta_target):
        self.kp = gains['kp']
        self.kd = gains['kd']
        self.r = theta_target

    def _hover_thrust(self, params):
        return params['m'] * params['g']

    def _gimbal(self, state, params, thrust):
        """Return (u_delta, thetaddot_des, delta_cmd) from the inner-loop law.

        Stateless (θ̇ measured), so it is exact to replay post-hoc. delta_cmd is
        the pre-mechanical-clip demand; u_delta is after the ±δ_max travel clip.
        """
        theta = state[4]
        thetadot = state[5]

        # Control effectiveness: b = T·L/I relates sin(δ) to θ̈ via I·θ̈ = -T·L·sin(δ)
        b = thrust * params['L'] / params['I']

        thetaddot_des = self.kp * (self.r - theta) - self.kd * thetadot
        # Exact inversion δ = arcsin(-θ̈_des / b); clip guards the arcsin domain
        delta_cmd = np.arcsin(np.clip(-thetaddot_des / b, -1, 1))
        u_delta = np.clip(delta_cmd, -params['delta_max'], params['delta_max'])
        return u_delta, thetaddot_des, delta_cmd

    def __call__(self, t, state, params):
        thrust = self._hover_thrust(params)
        u_delta, _, _ = self._gimbal(state, params, thrust)
        return thrust, u_delta

    def record(self, state, params):
        thrust = self._hover_thrust(params)
        u_delta, thetaddot_des, delta_cmd = self._gimbal(state, params, thrust)
        rec = _blank_record()
        rec.update({
            'theta_cmd': self.r,
            'u_T': np.clip(thrust, params['T_min'], params['T_max']),
            'delta': u_delta,
            'T_cmd': thrust,
            'delta_cmd': delta_cmd,
            'thetaddot_des': thetaddot_des,
        })
        return rec


class CascadedController:
    """Three-loop cascade: horizontal position (outer) → pitch attitude (middle) → TVC angle (inner).

    Each loop generates a reference for the next:
      1. Horizontal PD  →  pitch reference θ_ref  (via small-angle linearisation of the horizontal EOM)
      2. Altitude PD    →  thrust command T         (with gravity feedforward)
      3. Attitude PD    →  gimbal angle δ           (via exact inversion of the rotational EOM)

    Validity assumption: bandwidth separation ω_x << ω_θ, so the attitude loop tracks θ_ref
    fast enough to appear quasi-static from the horizontal loop's perspective.
    All PD loops use velocity/rate measurements on the derivative term (derivative-on-measurement).
    No integral action — steady-state z error is eliminated by feedforward; steady-state x error
    is rejected by gain tuning alone.
    """

    def __init__(self, gains, x_target, z_target):
        self.kp_x = gains['kp_x']
        self.kd_x = gains['kd_x']
        self.kp_z = gains['kp_z']
        self.kd_z = gains['kd_z']
        self.kp_theta = gains['kp_theta']
        self.kd_theta = gains['kd_theta']

        self.r_z = z_target
        self.r_x = x_target

    def commanded_tilt(self, state, params):
        # ── Outer loop: horizontal position → pitch reference ──────────────────
        # Small-angle linearisation of horizontal EOM: ẍ ≈ -g·θ (for |θ|, |δ| << 1).
        # Invert to map desired ẍ to a pitch setpoint: θ_ref = -ẍ_des / g.
        # State vector: [x, z, ẋ, ż, θ, θ̇]
        x = state[0]
        xdot = state[2]

        g = params['g']
        theta_max = params['tilt_limit']

        e_x = self.r_x - x
        xddot_des = self.kp_x * e_x - self.kd_x * xdot  # PD in acceleration space
        theta_des = - xddot_des / g
        r_theta = np.clip(theta_des, - theta_max, theta_max)
        return r_theta, xddot_des

    def commanded_thrust(self, state, params):
        # State vector: [x, z, ẋ, ż, θ, θ̇]
        z = state[1]
        zdot = state[3]
        theta = state[4]

        m = params['m']
        g = params['g']
        thrust_min = params['T_min']
        thrust_max = params['T_max']

        # ── Middle loop: altitude → thrust ─────────────────────────────────────
        e_z = self.r_z - z

        # Exact hover feedforward at tilt θ: vertical thrust component is T·cos(θ), so T_ff = mg/cos(θ).
        # However by linearising we remove the m, and put it back after
        # Uses θ rather than the net gimbal angle (θ−δ) — valid approximation for small δ.
        feedforward_z = g / np.cos(theta)
        # cos(θ) → 0 as |θ| → 90°; TVC saturation keeps this well-conditioned in practice.

        zddot_des = self.kp_z * e_z - self.kd_z * zdot + feedforward_z
        thrust_des = zddot_des * m
        u_thrust = np.clip(thrust_des, thrust_min, thrust_max)  # actuator saturation applied before attitude inversion
        # thrust_des returned alongside u_thrust so callers can see the pre-saturation demand
        return u_thrust, zddot_des, thrust_des

    def commanded_thrust_vector(self, state, params, r_theta, u_thrust):
        # State vector: [x, z, ẋ, ż, θ, θ̇]
        theta = state[4]
        thetadot = state[5]

        L = params['L']
        I = params['I']

        delta_max = params['delta_max']

        # ── Inner loop: pitch attitude → TVC angle ─────────────────────────────
        # Rotational EOM: I·θ̈ = -thrust·L·sin(δ)  →  control effectiveness b = thrust·L/I.
        # u_thrust (post-saturation) is used so that b reflects the torque actually available.
        b = u_thrust * L / I

        e_theta = r_theta - theta
        # PD law specifies desired angular acceleration
        thetaddot_des = self.kp_theta * e_theta - self.kd_theta * thetadot

        # Exact inversion of rotational EOM: δ = arcsin(-θ̈_des / b)
        # Clip to [-1, 1] guards the arcsin domain; applies when θ̈_des exceeds available authority b
        delta_des = np.arcsin(np.clip(- thetaddot_des / b, -1, 1))

        u_delta = np.clip(delta_des, - delta_max, delta_max)  # mechanical TVC travel limit
        # delta_des is the demand *after* the arcsin-domain clip (control-authority
        # saturation) but *before* the mechanical clip — the gap between it and
        # u_delta is therefore travel-limit saturation alone.
        return u_delta, thetaddot_des, delta_des

    def __call__(self, t, state, params):
        r_theta, xddot_des = self.commanded_tilt(state, params)
        u_thrust, zddot_des, T_des = self.commanded_thrust(state, params)
        u_delta, thetaddot_des, delta_des = self.commanded_thrust_vector(
            state, params, r_theta=r_theta, u_thrust=u_thrust
        )
        return u_thrust, u_delta

    def record(self, state, params):
        r_theta, xddot_des = self.commanded_tilt(state, params)
        u_thrust, zddot_des, thrust_cmd = self.commanded_thrust(state, params)
        u_delta, thetaddot_des, delta_cmd = self.commanded_thrust_vector(
            state, params, r_theta, u_thrust
        )
        return {
            'theta_cmd': r_theta,
            'u_T': u_thrust,
            'delta': u_delta,
            'T_cmd': thrust_cmd,
            'delta_cmd': delta_cmd,
            'xddot_des': xddot_des,
            'zddot_des': zddot_des,
            'thetaddot_des': thetaddot_des,
        }


class LQRController:
    def __init__(self, gains, x_target, z_target, theta_target):
        self.targets = gains
        self.reference_vector = np.array([x_target, z_target, 0, 0, 0, 0])  # Landing terminal conditions,
        # theta_target is ignored
        self._K = None

    def _compute_gain_matrix(self, params):
        A, B = linearize_hover(params)
        Q, R = bryson_weights(self.targets)
        K, *_ = control.lqr(A, B, Q, R)
        self._K = K
        return self._K

    def control_law(self, state, params):
        self._K = self._compute_gain_matrix(params)  # This now recomputes the gain matrix every time
        e = np.asarray(state[:6]) - self.reference_vector
        du = - (self._K @ e)
        thrust = params['m'] * params['g'] + du[0]  # du only accounts for the deviation from hover,
        # we here restore the full thrust command
        delta = du[1]
        return thrust, delta

    def __call__(self, t, state, params):
        thrust, delta = self.control_law(state, params)
        return thrust, delta

    def record(self, state, params):
        thrust_cmd, delta_cmd = self.control_law(state, params)
        rec = _blank_record()
        rec.update({
            'T_cmd': thrust_cmd,
            'delta_cmd': delta_cmd,
            # Mirror sim.py's closed_loop_rhs clip exactly, so the gap between the
            # *_cmd and post-clip values is the saturation diagnostic.
            'u_T': np.clip(thrust_cmd, params['T_min'], params['T_max']),
            'delta': np.clip(delta_cmd, -params['delta_max'], params['delta_max']),
        })
        return rec


# Registry of selectable controllers. Each carries their gains with some defaults, and their build lambda
CONTROLLER_REGISTRY = {
    "Cascaded PD": {
        "gain_fields": ["kp_x", "kd_x", "kp_z", "kd_z", "kp_theta", "kd_theta"],
        "defaults": {
            "kp_x": 0.16, "kd_x": 0.8,
            "kp_z": 0.16, "kd_z": 0.8,
            "kp_theta": 16.0, "kd_theta": 6.4,
        },
        "build": lambda gains, x_target, z_target, theta_target:
        CascadedController(gains, x_target, z_target),
    },
    "Altitude PID": {
        "gain_fields": ["kp", "ki", "kd"],
        "defaults": {"kp": 3.0, "ki": 0.0, "kd": 30.0},
        "build": lambda gains, x_target, z_target, theta_target:
        AltitudePIDController(gains, z_target),
    },
    "Attitude PD (inner-loop demo)": {
        "gain_fields": ["kp", "kd"],
        "defaults": {"kp": 16.0, "kd": 6.4},
        "build": lambda gains, x_target, z_target, theta_target:
        AttitudePDController(gains, theta_target),
    },
    "LQR": {
        "gain_fields": ["x_dev", "z_dev", "xdot_dev", "zdot_dev", "theta_dev", "thetadot_dev", "thrust_dev", "delta_dev"],
        "defaults": {"x_dev": 20.0,
                     "z_dev": 10.0,
                     "xdot_dev": 10.0,
                     "zdot_dev": 2.0,
                     "theta_dev": 0.1745,
                     "thetadot_dev": 0.3,
                     "thrust_dev": 750.0,
                     "delta_dev": 0.2094, },
        "build": lambda gains, x_target, z_target, theta_target:
        LQRController(gains, x_target, z_target, theta_target),
    },
}
