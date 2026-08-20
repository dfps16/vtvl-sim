import os
import shutil

import matplotlib as mpl
import numpy as np
from matplotlib import animation, font_manager
from matplotlib.figure import Figure
from matplotlib.patches import Polygon
from scipy.integrate import cumulative_trapezoid

from vtvl_sim.post_processing import engine_signals

# Figure palette (white/black + a single bright-red accent). White carries
# data/structure, red is reserved for key markers (targets, touchdown, thrust).
# The app chrome uses teal instead — red reads as an alert in an interface, but
# on a black plot it is the strongest available accent against white data.
_WHITE = '#FFFFFF'
_BLACK = '#000000'
_RED = '#FF2D2D'

# Adobe renamed the family "Source Sans 3" in 2021, so the same typeface is
# installed under either name depending on vintage. Resolve it once here rather
# than listing both in font.family — matplotlib logs a "font family not found"
# miss for every name it falls through, on every figure it draws.
_INSTALLED = {f.name for f in font_manager.fontManager.ttflist}
_SANS = next((n for n in ('Source Sans Pro', 'Source Sans 3') if n in _INSTALLED), 'DejaVu Sans')

mpl.rcParams.update({
    # DejaVu Sans backs up the stack: Source Sans has no θ/θ̇ glyphs.
    'font.family': [_SANS, 'DejaVu Sans'],
    'figure.facecolor': _BLACK,
    'axes.facecolor': _BLACK,
    'savefig.facecolor': _BLACK,
    'axes.edgecolor': _WHITE,
    'axes.labelcolor': _WHITE,
    'axes.titlecolor': _WHITE,
    'xtick.color': _WHITE,
    'ytick.color': _WHITE,
    'text.color': _WHITE,
    'grid.color': _WHITE,
    'grid.alpha': 0.15,
    'legend.facecolor': _BLACK,
    'legend.edgecolor': _WHITE,
    'legend.labelcolor': _WHITE,
})


def plot_state(sim_results, sim_setup):

    t = sim_results['t']
    x = sim_results['x']
    z = sim_results['z']
    xdot = sim_results['xdot']
    zdot = sim_results['zdot']
    theta = sim_results['theta']
    thetadot = sim_results['thetadot']

    fig = Figure(figsize=(12, 8))
    axes = fig.subplots(2, 3, sharex=True).flatten()
    
    axes[0].plot(t, x, label='x', color=_WHITE)
    # axes[0].axhline(x_target, color='r', linestyle='--', linewidth=0.8, label='x_target')
    axes[0].set_ylabel('x [m]')
    axes[0].set_title('Lateral position over time')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, z, label='z', color=_WHITE)
    # axes[2].axhline(z_target, color='r', linestyle='--', linewidth=0.8, label='z_target')
    axes[1].set_ylabel('z [m]')
    axes[1].set_title('Vertical position over time')
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t, np.degrees(theta), label='θ', color=_WHITE)
    # axes[2].plot(t, np.degrees(theta_cmd), linestyle='--', label='θ_cmd')
    axes[2].set_ylabel('θ [deg]')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_title('Attitude over time')
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(t, xdot, color=_WHITE)
    axes[3].set_ylabel('ẋ [m/s]')
    axes[3].set_title('Lateral velocity over time')
    axes[3].grid(True)

    axes[4].plot(t, zdot, color=_WHITE)
    axes[4].set_ylabel('ż [m/s]')
    axes[4].set_title('Vertical velocity over time')
    axes[4].grid(True)

    axes[5].plot(t, np.degrees(thetadot), color=_WHITE)
    axes[5].set_ylabel('θ̇ [deg/s]')
    axes[5].set_xlabel('Time [s]')
    axes[5].set_title('Angular rate over time')
    axes[5].grid(True)

    fig.suptitle('Vehicle State', fontsize=18, fontweight='bold')
    fig.tight_layout()
    return fig

def plot_trajectory(sim_results):

    x = sim_results['x']
    z = sim_results['z']

    # error = np.abs(x_target - x[-1])
    # xdot_f = xdot[-1]
    # zdot_f = zdot[-1]
    # vel_abs = np.sqrt(xdot_f ** 2 + zdot_f ** 2)
    # theta_f = np.degrees(theta[-1])

    fig = Figure(figsize=(4, 8))
    ax = fig.subplots()
    
    ax.plot(x, z, color=_WHITE)
    ax.plot(x[0], z[0], 'o', color=_WHITE, label='start')
    ax.plot(x[-1], z[-1], 's', color=_RED, label='end')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('z [m]')
    ax.set_title('Trajectory (z vs x)')
    ax.legend(loc='upper right')
    ax.grid(True)
    # ax.set_aspect('equal')

    fig.tight_layout()
    return fig

def plot_engine(sim_results, sim_setup):
    """Engine state: thrust/throttle, gimbal angle, and thrust vector direction.

    Commanded vs actual differ only where an actuator limit bites — there is no
    engine lag in this model, so thrust is applied instantaneously. The visible
    gap between the two traces is therefore exactly the saturation.

    Note on the gimbal row: δ_cmd has already passed through the arcsin-domain
    clip in CascadedController.commanded_thrust_vector, which is *control-authority*
    saturation (θ̈_des beyond what the current thrust can deliver). The gap drawn
    here is the mechanical ±δ_max travel limit only.
    """
    t = sim_results['t']
    theta = sim_results['theta']
    delta = sim_results['delta']
    delta_cmd = sim_results['delta_cmd']

    params = sim_setup['params']
    T_max = params['T_max']
    T_min = params['T_min']
    T_hover = sim_results['m'] * params['g']  # tracks instantaneous mass, not a fixed reference
    delta_max_deg = np.degrees(params['delta_max'])

    T_cmd, T_act, _, _ = engine_signals(sim_results, sim_setup)

    fig = Figure(figsize=(10, 9))
    axes = fig.subplots(3, 1, sharex=True)

    # ── Thrust & throttle ──────────────────────────────────────────────────
    axes[0].plot(t, T_cmd, linestyle='--', color=_RED, label='T_cmd (demand)')
    axes[0].plot(t, T_act, color=_WHITE, linewidth=1.5, label='T (applied)')
    axes[0].axhline(T_max, color=_WHITE, linestyle=':', linewidth=0.8, alpha=0.35, label='T_min / T_max')
    axes[0].axhline(T_min, color=_WHITE, linestyle=':', linewidth=0.8, alpha=0.35)
    axes[0].plot(t, T_hover, color=_WHITE, linestyle='-.', linewidth=0.8, alpha=0.25, label='hover (mg)')
    # Shade the spans where the demand is outside the throttle box — i.e. where
    # the controller asked for thrust the engine could not deliver.
    axes[0].fill_between(t, T_min, T_max, where=(T_cmd > T_max) | (T_cmd < T_min),
                         color=_RED, alpha=0.12, label='saturated')
    axes[0].set_ylabel('Thrust [N]')
    axes[0].set_title('Thrust: commanded vs actual')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True)

    # Throttle is exactly T / T_max, so a locked twin axis shows it without
    # replotting the same curve at a different scale.
    thr_ax = axes[0].twinx()
    lo, hi = axes[0].get_ylim()
    thr_ax.set_ylim(100.0 * lo / T_max, 100.0 * hi / T_max)
    thr_ax.set_ylabel('Throttle [%]')
    thr_ax.tick_params(colors=_WHITE)

    # ── Gimbal angle ───────────────────────────────────────────────────────
    axes[1].plot(t, np.degrees(delta_cmd), linestyle='--', color=_RED, label='δ_cmd (demand)')
    axes[1].plot(t, np.degrees(delta), color=_WHITE, linewidth=1.5, label='δ (applied)')
    axes[1].axhline(delta_max_deg, color=_WHITE, linestyle=':', linewidth=0.8, alpha=0.35, label='±δ_max')
    axes[1].axhline(-delta_max_deg, color=_WHITE, linestyle=':', linewidth=0.8, alpha=0.35)
    axes[1].fill_between(t, -delta_max_deg, delta_max_deg,
                         where=np.abs(np.degrees(delta_cmd)) > delta_max_deg,
                         color=_RED, alpha=0.12, label='saturated')
    axes[1].set_ylabel('δ [deg]')
    axes[1].set_title('Thrust vector angle (gimbal, body-relative)')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True)
    # δ_cmd runs to ±90° when thrust collapses to T_min and the arcsin clip bites
    # (total loss of gimbal authority). Left autoscaled that swamps the ±δ_max
    # detail, so bound the view to the travel limit and let the demand run off
    # the top — the shaded span still marks where it happened.
    axes[1].set_ylim(-2.0 * delta_max_deg, 2.0 * delta_max_deg)

    # ── Thrust direction in the inertial frame ─────────────────────────────
    # lander_eom projects thrust through (theta - delta), so this is the angle
    # that actually steers the force vector. The gap from θ is the gimbal's doing.
    axes[2].plot(t, np.degrees(theta - delta), color=_WHITE, linewidth=1.5, label='θ − δ (thrust direction)')
    axes[2].plot(t, np.degrees(theta), color=_WHITE, linestyle=':', linewidth=1.0, alpha=0.5, label='θ (body attitude)')
    axes[2].set_ylabel('angle from vertical [deg]')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_title('Thrust direction (inertial)')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(True)

    fig.suptitle('Engine State', fontsize=18, fontweight='bold')
    fig.tight_layout()
    return fig


def plot_propellant(sim_results, sim_setup):
    """Mass depletion and cumulative impulse, with a built-in cross-check.

    Two independent ways of tracking propellant use should agree: the ODE
    integrator's actual mass loss (state[6], panel 1) and the post-hoc integral
    of applied thrust via the Tsiolkovsky relation (panel 2, right axis). They
    are the same physical integral computed two different ways — divergence
    between the two curves in panel 2 would flag a bug, not a modelling choice
    (PLAN.md §2.7).
    """
    t = sim_results['t']
    m = sim_results['m']
    thrust = sim_results['u_T']

    params = sim_setup['params']
    m_dry = params['m_dry']
    g = params['g']
    isp = params['isp']

    fig = Figure(figsize=(10, 7))
    axes = fig.subplots(2, 1, sharex=True)

    # ── Mass vs time ─────────────────────────────────────────────────────────
    axes[0].plot(t, m, color=_WHITE, linewidth=1.5, label='m (actual)')
    axes[0].axhline(m_dry, color=_WHITE, linestyle=':', linewidth=0.8, alpha=0.35, label='m_dry')
    if m[-1] <= m_dry + 1e-6:
        axes[0].plot(t[-1], m[-1], 'x', color=_RED, markersize=10, label='flameout')
    axes[0].set_ylabel('m [kg]')
    axes[0].set_title('Mass depletion over time')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True)

    # ── Cumulative impulse vs time, with propellant cross-check ────────────────
    impulse_cum = cumulative_trapezoid(thrust, t, initial=0.0)
    propellant_actual = m[0] - m  # actual mass consumed so far, same sign convention

    axes[1].plot(t, impulse_cum, color=_WHITE, linewidth=1.5, label='∫T dt (impulse)')
    axes[1].set_ylabel('Impulse [N·s]')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_title('Cumulative impulse, with post-hoc vs. actual propellant cross-check')
    axes[1].grid(True)

    # Locked twin axis: impulse / (g*isp) is exactly the post-hoc propellant
    # estimate, so the same curve reads directly in kg on the right.
    prop_ax = axes[1].twinx()
    lo, hi = axes[1].get_ylim()
    prop_ax.set_ylim(lo / (g * isp), hi / (g * isp))
    prop_ax.set_ylabel('Propellant [kg]')
    prop_ax.tick_params(colors=_WHITE)
    prop_ax.plot(t, propellant_actual, color=_RED, linestyle='--', linewidth=1.0,
                 alpha=0.7, label='propellant consumed (actual)')

    lines_1, labels_1 = axes[1].get_legend_handles_labels()
    lines_2, labels_2 = prop_ax.get_legend_handles_labels()
    axes[1].legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper left', fontsize=8)

    fig.suptitle('Propellant Usage', fontsize=18, fontweight='bold')
    fig.tight_layout()
    return fig


def _resample(t, arrays, fps, playback_speed):
    """Interpolate the (non-uniform) integrator output onto a uniform frame grid.

    solve_ivp uses adaptive steps, so playing raw samples at fixed fps would
    distort timing. We build an even time base and linearly interpolate.

    playback_speed is sim-seconds shown per real second: 1.0 = real time,
    <1.0 = slow motion (more frames), >1.0 = time-lapse.
    """
    dt_frame = playback_speed / fps
    t_uniform = np.arange(t[0], t[-1], dt_frame)
    resampled = [np.interp(t_uniform, t, a) for a in arrays]
    return t_uniform, resampled


def _glyph(cx, cz, theta, delta, throttle_frac, glyph_len):
    """Return drawing primitives for the lander at one instant.

    Body 'up'/nose unit vector n = (-sin theta, cos theta) — consistent with the
    thrust direction in dynamics.lander_eom. Lateral unit p = (cos theta, sin theta).
    The exhaust plume points opposite the gimballed thrust axis (theta - delta).
    """
    half = glyph_len / 2.0
    width = glyph_len * 0.35
    leg_len = glyph_len * 0.55
    flame_base = glyph_len * 0.35
    flame_gain = glyph_len * 1.5

    n = np.array([-np.sin(theta), np.cos(theta)])   # body axis, nose direction
    p = np.array([np.cos(theta), np.sin(theta)])    # body lateral axis
    C = np.array([cx, cz])

    nose = C + half * n
    base = C - half * n

    # Body rectangle corners (closed polygon)
    body = np.array([
        nose + (width / 2) * p,
        nose - (width / 2) * p,
        base - (width / 2) * p,
        base + (width / 2) * p,
    ])

    # Two splayed landing legs from the base, drawn as one broken line
    dir_l = (-n - p)
    dir_l = dir_l / np.linalg.norm(dir_l)
    dir_r = (-n + p)
    dir_r = dir_r / np.linalg.norm(dir_r)
    foot_l = base + leg_len * dir_l
    foot_r = base + leg_len * dir_r
    legs_x = [base[0], foot_l[0], np.nan, base[0], foot_r[0]]
    legs_z = [base[1], foot_l[1], np.nan, base[1], foot_r[1]]

    # Exhaust plume: opposite the thrust direction (-sin(theta-delta), cos(theta-delta))
    thrust_dir = np.array([-np.sin(theta - delta), np.cos(theta - delta)])
    plume_dir = -thrust_dir
    flame_len = flame_base + flame_gain * throttle_frac
    flame_tip = base + flame_len * plume_dir
    flame_x = [base[0], flame_tip[0]]
    flame_z = [base[1], flame_tip[1]]

    return body, (legs_x, legs_z), (flame_x, flame_z)



def animate_descent(sim_results, sim_setup,
                    fps=20, playback_speed=1.0, glyph_len=8.0,
                    save_path=None):
    """Render the trajectory to a video file.

    Returns (fig, anim, out_path). out_path is the file actually written, which
    is not necessarily save_path: without ffmpeg we fall back to a GIF. It is
    None when save_path is None.

    targets: list of (x, z) waypoints to mark (one per mission phase).
    """
    t = sim_results['t']
    x = sim_results['x']
    z = sim_results['z']
    xdot = sim_results['xdot']
    zdot = sim_results['zdot']
    theta = sim_results['theta']
    delta = sim_results['delta']

    _, _, thr_cmd, thr_act = engine_signals(sim_results, sim_setup)
    speed = np.sqrt(xdot ** 2 + zdot ** 2)

    targets = [(x_t, z_t) for x_t, z_t, *_ in sim_setup['phases']]

    t_u, (x_u, z_u, th_u, dl_u, act_u, cmd_u, sp_u) = _resample(
        t, [x, z, theta, delta, thr_act, thr_cmd, speed], fps, playback_speed
    )
    z_u_pos = np.interp(t_u, t, z)  # altitude for HUD (same as z_u, kept explicit)

    # Fixed axis limits (equal aspect) from trajectory + target extents, padded.
    tx = [p[0] for p in targets]
    tz = [p[1] for p in targets]
    pad = glyph_len * 1.5
    x_lo = min(x.min(), *tx) - pad
    x_hi = max(x.max(), *tx) + pad
    z_hi = max(z.max(), *tz) + pad
    z_lo = -pad

    # A VTVL descent is naturally tall and thin, which under equal aspect gives
    # a sliver of a frame. Clamp the *view* aspect by widening the limits rather
    # than squashing the data, so the axes box below can match the data aspect
    # exactly and equal aspect costs no blank space.
    min_aspect, max_aspect = 0.35, 1.6
    aspect = (x_hi - x_lo) / (z_hi - z_lo)
    if aspect < min_aspect:
        grow = min_aspect * (z_hi - z_lo) - (x_hi - x_lo)
        x_lo -= grow / 2.0
        x_hi += grow / 2.0
    elif aspect > max_aspect:
        z_hi += (x_hi - x_lo) / max_aspect - (z_hi - z_lo)  # grow up; z_lo is the ground
    x_range = x_hi - x_lo
    z_range = z_hi - z_lo

    # Lay the figure out in inches. The axes box is sized to the data aspect, so
    # set_aspect('equal') fits it exactly rather than shrinking it and baking the
    # leftover into every frame as blank space. Margins are only as deep as the
    # labels need; the HUD gets a strip of its own so it can't overlap the plot.
    m_left, m_right = 1.0, 0.15   # z tick labels + 'z [m]' / trailing padding
    m_top, m_bottom = 0.55, 0.65  # title + legend strip / x tick labels + 'x [m]'
    hud_gap, hud_width = 0.25, 2.6

    axes_height = 8.0
    axes_width = axes_height * (x_range / z_range)
    fig_width = m_left + axes_width + hud_gap + hud_width + m_right
    fig_height = m_bottom + axes_height + m_top

    fig = Figure(figsize=(fig_width, fig_height), dpi=150)
    ax = fig.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(z_lo, z_hi)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('z [m]')
    ax.set_title('VTVL trajectory', fontsize=9)
    ax.grid(True, alpha=0.15)

    fig.subplots_adjust(
        left=m_left / fig_width,
        right=(m_left + axes_width) / fig_width,
        bottom=m_bottom / fig_height,
        top=(m_bottom + axes_height) / fig_height,
    )

    # Static scene
    ax.axhline(0.0, color=_WHITE, linewidth=2, alpha=0.4, zorder=0)      # ground
    for k, (tx_, tz_) in enumerate(targets):
        ax.plot(tx_, tz_, '*', color=_RED, markersize=16, zorder=1,
                label='target' if k == 0 else None)
    ax.plot(x[0], z[0], 'o', color=_WHITE, markersize=6, label='start', zorder=1)

    # Dynamic artists
    (trail,) = ax.plot([], [], '-', color=_WHITE, linewidth=1.0, alpha=0.7, label='path')
    (flame,) = ax.plot([], [], '-', color=_RED, linewidth=6, solid_capstyle='round', zorder=2)
    (legs,) = ax.plot([], [], '-', color=_WHITE, linewidth=2, zorder=3)
    body = Polygon(np.zeros((4, 2)), closed=True, facecolor=_WHITE,
                   edgecolor=_RED, linewidth=1.2, zorder=4)
    ax.add_patch(body)
    # Figure-level (not axes-level) text, placed in the reserved right-hand
    # strip — this is what keeps it fully outside the trajectory plot itself,
    # rather than just repositioned within the same axes. Anchored to the top of
    # the axes box so it reads alongside the plot instead of floating mid-strip.
    hud = fig.text((m_left + axes_width + hud_gap) / fig_width,
                   (m_bottom + axes_height) / fig_height,
                   '', transform=fig.transFigure, va='top', ha='left',
                   family='monospace', fontsize=8,
                   bbox=dict(boxstyle='round', facecolor=_BLACK, alpha=0.9, edgecolor=_RED))
    # Legend placed above and to the right of the axes (outside the plot
    # area) so it never overlaps the trajectory/lander content, including
    # near apogee which sits roughly centered in x.
    ax.legend(loc='lower right', bbox_to_anchor=(1.0, 1.02), ncol=3,
              fontsize=8, frameon=False)

    def update(i):
        # Plume length tracks the actual (applied) throttle — the physical thrust.
        body_xy, (lx, lz), (fx, fz) = _glyph(
            x_u[i], z_u[i], th_u[i], dl_u[i], act_u[i], glyph_len
        )
        body.set_xy(body_xy)
        legs.set_data(lx, lz)
        flame.set_data(fx, fz)
        trail.set_data(x_u[:i + 1], z_u[:i + 1])
        hud.set_text(
            f't      = {t_u[i]:6.2f} s\n'
            f'alt    = {z_u_pos[i]:6.1f} m\n'
            f'speed  = {sp_u[i]:6.2f} m/s\n'
            f'theta  = {np.degrees(th_u[i]):6.2f} deg\n'
            f'thrott   cmd={cmd_u[i] * 100:6.1f} %  act={act_u[i] * 100:6.1f} %'
        )
        return body, legs, flame, trail, hud

    anim = animation.FuncAnimation(
        fig, update, frames=len(t_u), interval=1000.0 / fps, blit=False
    )

    out = None
    if save_path is not None:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        if shutil.which('ffmpeg'):
            writer = animation.FFMpegWriter(fps=fps, bitrate=4000)
            out = save_path
        else:
            print('[animate_descent] ffmpeg not found — falling back to GIF.')
            writer = animation.PillowWriter(fps=fps)
            out = os.path.splitext(save_path)[0] + '.gif'
        anim.save(out, writer=writer)
        print(f'[animate_descent] wrote {out}  ({len(t_u)} frames @ {fps} fps)')
    return fig, anim, out