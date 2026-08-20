# 2D VTVL Descent Simulation

Planar (3-DOF) thrust-vectoring lander simulation with cascaded PD, LQR, and (stretch) convex G-FOLD guidance.

This repository is the **simulation engine** — an importable `vtvl_sim` package plus a CLI. It has no GUI. The interactive Starworks front-end lives in its own repo and consumes this engine as a pinned dependency:

> **GUI:** [SUSF-Starworks/lander-trajectory-tool](https://github.com/SUSF-Starworks/lander-trajectory-tool)

---

## Problem

A rigid-body lander in a vertical plane has state `(x, z, ẋ, ż, θ, θ̇)` and two controls: thrust magnitude `T` and gimbal angle `δ`. The plant is underactuated — three translational/rotational DOF driven by two inputs. Horizontal translation is controlled through attitude, exactly as in a quadrotor. The goal is a soft, accurate touchdown from an offset initial condition, with honest comparison of controllers on fuel, accuracy, and constraint satisfaction.

### Equations of motion

```
m ẍ  =  -T sin(θ - δ)
m z̈  =   T cos(θ - δ) - mg
I θ̈  =  -T sin(δ) · L
```

`θ` is pitch from vertical, `δ` is gimbal deflection from body axis, `L` is the moment arm from CoM to gimbal pivot. Sign convention verified against free-body diagram — the torque opposes positive `δ` and the lateral force coupling is consistent.

---

## Repository layout

```
vtvl-sim/
├── pyproject.toml        — project metadata + pinned dependencies (uv-managed)
├── uv.lock               — resolved dependency lockfile for reproducible installs
├── test_scenarios/
│   ├── default.json      — canonical defaults, single source of truth for params
│   ├── scenario1.json    — example scenario: physical params, controller/gains, phases, outputs
│   └── lqr1.json         — LQR head-to-head scenario, same geometry as default.json
├── src/
│   └── vtvl_sim/
│       ├── __init__.py         — public API surface (see "Using the engine")
│       ├── params.py          — reference defaults for notebooks + tests (runtime config is JSON-driven)
│       ├── paths.py           — centralised results-directory paths (no hardcoded absolute paths)
│       ├── dynamics.py        — 3-DOF EOM over a 7-state vector (mass depletion via the Tsiolkovsky rate)
│       ├── sim.py             — vtvl_solver/sim_run: solve_ivp wrapper, phase chaining, touchdown + propellant-exhaustion events
│       ├── controllers.py     — Altitude PID, Attitude PD (inner-loop demo), Cascaded PD, LQR (gain-scheduled on mass) — all in CONTROLLER_REGISTRY
│       ├── schemas.py         — Pydantic models validating scenario JSON files
│       ├── scenario_io.py     — load_scenario / build_setup: JSON -> validated sim/solver/output setup
│       ├── post_processing.py — CSV export, touchdown/flameout report (with a post-hoc-vs-actual propellant cross-check), state/trajectory/engine/propellant metrics
│       ├── guidance.py        — convex G-FOLD reference (stretch, empty stub)
│       ├── run_scenarios.py   — CLI entrypoint: run a scenario JSON end to end
│       └── plotting.py        — state/trajectory/engine/propellant plots, descent animation (HUD includes propellant remaining)
├── notebooks/
│   ├── check_sim.py         — baseline altitude-PID diagnostics
│   ├── attitude_loop.py     — inner attitude-loop response + robustness
│   ├── check_attitude.py    — inner-loop verification against design targets
│   └── check_cascade.py     — full cascade divert-and-land scenario
├── results/              — saved diagnostic plots (generated, gitignored)
├── REPORT_NOTES.md       — running log of report-worthy findings, appended to as work proceeds
└── tests/
    ├── dynamics_test.py       — free-fall and mass-aware hover equilibrium
    ├── lqr_test.py            — controllability, closed-loop stability, gain structure, landing regression
    ├── mass_depletion_test.py — Tsiolkovsky mass conservation, propellant-exhaustion event, LQR gain-scheduling, both controllers under depletion
    ├── attitude_test.py       — inner-loop regression: steady-state error, overshoot vs. linear prediction, sign convention, gimbal saturation
    └── attitude_test_plan.md  — rationale/spec behind attitude_test.py
```

---

## Progress

### Week 1 — dynamics + simulator (complete)

**`dynamics.py`** — `lander_eom(t, state, T, delta, params)` implements the full 3-DOF nonlinear EOM. State vector `[x, z, ẋ, ż, θ, θ̇]`; params dict carries `m, I, L, g`.

**`sim.py`** — `run_sim` wraps `solve_ivp` with a terminal touchdown event (`z = 0`, descending). `closed_loop_rhs` queries the controller, saturates actuator commands to `[T_min, T_max]` and `±δ_max`, then calls the EOM.

**`controllers.py`** — `AltitudePIDController` (Baseline 0): 1-DOF hover/descent with `mg` feedforward. Gimbal fixed at zero; no lateral or attitude control. Sanity-checks the hover equilibrium and integrator.

**Tests** (both passing, logic re-verified via independent RK4 integration):
- `test_free_fall` — zero thrust reduces to analytic projectile motion, error < 1e-6 m
- `test_hover_equilibrium` — `T = mg, θ = δ = 0` holds state constant over 10 s, drift < 1e-4

**Closed-loop baseline result.** PID tuned to `kp=3.0, ki=0.0, kd=30.0` with conditional integration (integral active only when `|e| < 5`). From a 100 m drop at rest, the lander reaches a soft vertical touchdown at **ż ≈ -0.66 m/s** in ≈26 s — vertical channel confirmed working before adding lateral/attitude control.

> **Resolved in Week 2:** the earlier split between `tests/dynamics_test.py` (`m=1500, I=2000, L=1.5`) and the closed-loop sim (`m=120, I=200, L=0.5`) is gone — both now import a single `src/params.py` source of truth.

### Week 2 — cascaded PD (substantially complete)

Parameters centralised into `src/params.py` (physical constants, gains, and design targets), so the simulator, notebooks, and tests can no longer drift apart. `sim.py` now takes the controller as an explicit argument rather than through the params dict.

**`AttitudePDController`** (inner loop) — PD in angular-acceleration space with dynamic inversion. From the rotational EOM `I·θ̈ = -T·L·sin(δ)`, a PD law sets `θ̈_des`, inverted exactly to `δ = arcsin(-θ̈_des / b)` with `b = T·L/I`. Stateless (rate measured), with arcsin-domain clipping and hard δ saturation.

**`CascadedController`** — three nested loops, each generating the next loop's reference:
- *Outer (position → tilt):* horizontal PD → pitch reference `θ_ref = -ẍ_des / g` (small-angle inversion), clipped to `tilt_limit`.
- *Middle (altitude → thrust):* PD with exact hover feedforward `mg/cos(θ)`, saturated to `[T_min, T_max]`.
- *Inner (attitude → gimbal):* EOM inversion as above, using post-saturation thrust for `b`.

Gains are derived from design targets `(ζ, ωₙ)` assuming an ideal 2nd-order plant (`kp = ωₙ²`, `kd = 2ζωₙ`), so retuning happens at the physics level. Bandwidth separation ω_x ≪ ω_θ (0.4 vs 4 rad/s) keeps the inner loop quasi-static from the outer loop's perspective.

**Divert-and-land scenario running** (`notebooks/check_cascade.py`): from 100 m altitude with a 20 m lateral offset target, the cascade drives a soft vertical touchdown, producing position/attitude tracking, trajectory (z-vs-x), rate, and control-input plots plus CSV export.

**Remaining:** write the inner-loop regression suite (`tests/attitude_test_plan.md` specs it — steady-state error, overshoot vs linear prediction, sign convention, saturation) and extend regression coverage to the closed-loop lateral channel.

### Week 3 — LQR (complete)

`linearize_hover()` and `bryson_weights()` in `controllers.py` implement the trim-point
Jacobian linearisation and Bryson's-rule cost weighting; `LQRController` is registered in
`CONTROLLER_REGISTRY` with empirically-tuned (not textbook-default) gains — the naive
Bryson defaults crash the vehicle, see `REPORT_NOTES.md` §4.2 for why. Gated by a controllability
check (`rank(ctrb(A,B)) == 6`) and closed-loop stability (`Re(eig(A−BK)) < 0`), both
covered by `tests/lqr_test.py` (4 tests, all passing) alongside a closed-form gain-structure
check and a landing-survivability regression on `test_scenarios/lqr1.json`.

Head-to-head against the cascade on identical geometry (100 m drop, 20 m lateral offset):

| | Cascaded PD | LQR (tuned) |
|---|---|---|
| Touchdown time | 17.6 s | 24.8 s |
| Touchdown ż | −0.35 m/s | **−0.20 m/s** |
| Lateral error | −0.35 m | **+0.14 m** |
| Peak \|θ\| | 10.1° | **7.0°** |
| Gimbal saturated | 3.4% | **0.0%** |

LQR trades speed for staying inside the linear/actuator-comfortable regime — see
`REPORT_NOTES.md` §6 for the full table and §1–3 for the theory (trim/cyclic
coordinates, the corrected lateral transfer function, the closed-form position-channel
gains, and why the two channels tune independently).

> **Resolved in Week 4:** the constant-mass assumption below is gone — mass is now a
> real state, and `LQRController` gain-schedules `K` on it every call instead of
> caching a single value.

### Week 4 — mass depletion (complete), guidance stretch + write-up (open)

Mass depletion is fully implemented: mass is a 7th state (`dynamics.py`), integrated via
the Tsiolkovsky rate `ṁ = -T/(g·Isp)`; `sim.py` injects the instantaneous mass into every
controller call and adds a `propellant_expended` terminal event (mirroring `touchdown`),
so a run that runs dry before landing is reported distinctly as a **flameout** rather than
misread as a touchdown. `LQRController` no longer caches `K` — it gain-schedules on the
live mass every call, while the three PD-based controllers needed **zero code changes**
(their feedforward already re-read `params['m']` fresh every step). A new propellant-usage
plot and an animation HUD readout track mass depletion visually; `write_sim_report` cross-
checks the post-hoc thrust-integral propellant estimate against the ODE's actual integrated
mass loss (they agree to <0.001 kg). Covered by `tests/mass_depletion_test.py` (4 tests)
plus updated `dynamics_test.py`, alongside the previously-unwritten
`tests/attitude_test.py` (4 tests) — full suite: **14/14 passing**.

**Report-worthy result:** under a realistic `Isp = 200 s` and a 10–20% propellant margin,
LQR's slow, low-thrust-effort strategy (§Week 3's headline: it spends time to save
actuator authority) burns *more total propellant* than the cascade despite lower
instantaneous thrust, and currently flames out short of touchdown on the same geometry the
cascade lands on cleanly. Not a bug — a direct, documented consequence of gains tuned at
constant mass; see `REPORT_NOTES.md` §7. A deliberate re-tune under the depleting-mass
model is the natural next step, deferred so the mechanism could be verified correct
first.

Still open: the guidance stretch choice — convex G-FOLD reference tracked by LQR, or
Monte Carlo dispersion analysis if skipping guidance — and the eventual re-tune above.
README becomes the short technical report once those settle.

---

## Parameters (nominal)

Runtime configuration is JSON-driven — a run's parameters come from its scenario file (`test_scenarios/*.json`), validated by `schemas.py`. `test_scenarios/default.json` holds the canonical set below, which downstream tools (the GUI) load for their defaults so the engine, CLI, and scenario files cannot drift. `src/vtvl_sim/params.py` carries the same values as reference defaults for the notebooks and dynamics tests.

| Symbol | Value | Description |
|--------|-------|-------------|
| `m_dry` | 200 kg | Dry mass (propellant excluded) |
| `I` | 200 kg·m² | Pitch moment of inertia (held constant — see below) |
| `L` | 0.5 m | CoM-to-gimbal moment arm |
| `g` | 9.81 m/s² | Gravitational acceleration |
| `Isp` | 200 s | Specific impulse, against local `g` (not standard `g₀` — a stated simplification) |
| `T_min` | 1000 N | Minimum throttle (0.4·T_max, non-zero) |
| `T_max` | 2500 N | Maximum thrust (≈1.27× hover weight at `m_dry`) |
| `δ_max` | 12° | Gimbal deflection limit |
| `tilt_limit` | 10° | Pitch reference clamp (outer-loop θ_cmd limit) |

`initial_state.m` (wet mass) sets the propellant margin per scenario — e.g. 220 kg against
`m_dry = 200 kg` is a 20 kg (10%) margin in `test_scenarios/lqr1.json`. Moment of inertia
`I` does not shrink as propellant burns (would need a propellant mass-distribution
assumption this project hasn't made elsewhere) — a stated simplification, not modelled.
See `REPORT_NOTES.md` §7 for how tight that margin turns out to be against the tuned
controllers' actual propellant use.

---

## Using the engine

`vtvl_sim` exposes a stable public API from the package root. Downstream tools import from `vtvl_sim` and never reach into submodules:

```python
from vtvl_sim import (
    sim_run, build_setup, load_scenario, CONTROLLER_REGISTRY,
    plot_trajectory, plot_state, plot_engine, animate_descent,
    compute_state_metrics, compute_trajectory_metrics, compute_engine_metrics,
    __version__,
)
```

This is the contract the GUI depends on. Any change a consumer needs is a new tagged engine release, and the consumer bumps its pin — the front-end pins the engine by tag, e.g.:

```toml
# in the GUI's pyproject.toml
dependencies = ["vtvl-sim @ git+https://github.com/dfps16/vtvl-sim.git@v0.3.0"]
```

**v0.3.0 is a breaking release for scenario files:** `ParamsSchema.m` is renamed to
`m_dry`, and `LanderState` gains a required `m` field (wet mass, must exceed `m_dry`).
Any GUI-side code that builds a `sim_setup`/`initial_state` dict directly needs both
changes before bumping past `v0.2.0`. `sim_run`'s results dict gains one new key (`'m'`,
the mass trajectory) — additive, not breaking.

---

## Setup

Requires [uv](https://docs.astral.sh/uv/):

```bash
uv sync
```

Run tests from the repo root:

```bash
uv run pytest tests/ -v
```

Run a scenario:

```bash
uv run python -m vtvl_sim.run_scenarios test_scenarios/scenario1.json
```
