# Inner Attitude Loop — Regression Test Architecture

Status: **implemented** — `tests/attitude_test.py`, 4/4 passing. This document
remains as the rationale/spec behind those tests.

## Purpose and scope

The existing `dynamics_test.py` validates the **open-loop plant** (free-fall,
hover equilibrium). This suite validates the **closed inner loop** — the
`AttitudePDController` (gimbal δ → pitch θ, PD + dynamic inversion) wrapped around
that plant — against the linear design targets.

It encodes the acceptance criteria agreed in Week 2 as automated pass/fail
assertions, so they are re-checked on every change rather than eyeballed once on
a plot. It is the executable counterpart of `notebooks/attitude_loop.py`.

In scope: the isolated inner loop at fixed throttle.
Out of scope: translational coupling, disturbance rejection, large-angle
behaviour beyond saturation, mass depletion.

## Test fixture

All cases use the same isolation as the harness:

- Throttle pinned at hover, `T = m·g`, so the vehicle floats while attitude is
  exercised. Lateral/vertical drift is irrelevant and ignored.
- Initial state level and at rest: `[0, 100, 0, 0, 0, 0]`.
- A pitch-step command `θ_cmd`, run with `solve_ivp` (`max_step = 0.005`).
- δ recomputed post-hoc from the solution states (controller is stateless, so
  this is exact).
- Gains derived from the design targets `SYS_PROP_ATT` (ζ, ωₙ) in `params.py`,
  the single source of truth, so the test tracks any retune automatically.

Two step sizes:
- **Small step (~1.5–2°)** — stays in the linear regime; used for the
  quantitative checks where linear-theory predictions must hold.
- **Large step (~8°)** — drives the gimbal into saturation; used for the
  actuator-limit check.

## Test cases

| # | Assertion | Step | Tolerance (provisional) | What it guards |
|---|-----------|------|-------------------------|----------------|
| 1 | Steady-state error: final θ ≈ θ_cmd | small | \|θ_end − θ_cmd\| < 0.05° | PD reaches target; no offset |
| 2 | Overshoot ≈ linear prediction `exp(−ζπ/√(1−ζ²))` | small | within ±0.5 pp of predicted | gains realise the intended ωₙ, ζ |
| 3 | Sign convention: first commanded δ is negative for +θ_cmd | small | `δ_first < 0` | flipped sign in EOM torque or inversion |
| 4 | Saturation + limit respect: δ reaches ±δ_max and never exceeds | large | `max\|δ\| == δ_max` (within 1e-3) | clip works; actuator model honoured |

Optional / lower priority:
- Settling time within tolerance of `4/(ζωₙ)`. Noisier under adaptive stepping —
  keep the tolerance generous or omit.
- Rise time monotonicity / finiteness.

Check **2** is the strongest: stability alone is a weak guarantee; matching the
predicted overshoot confirms the closed loop has the *designed* pole locations,
not just stable ones.

## Assumptions and known coupling

- **Constant mass, fixed throttle.** Linear-theory predictions (overshoot,
  settling) are only valid on the constant-mass plant. When Week 3 adds mass
  depletion, `b = T·L/I` shifts as propellant drains, ωₙ moves, and check 2's
  prediction must be updated or its tolerance widened. This is intended — a
  failure there will flag that the design point has moved.
- **Tolerances must absorb integrator sampling.** `solve_ivp` adaptive stepping
  means peak/settling samples jitter slightly between runs. Tolerances are set
  to catch real regressions (sign flips, gain/param drift, interface breaks)
  without failing on step-size noise. The overshoot check is robust at
  `max_step = 0.005`.
- **No disturbance rejection tested.** The nominal plant has no disturbance
  torque and the controller has no integral term, so there is no steady-state
  disturbance case to assert yet. Revisit when a CoM offset or wind model is
  introduced.

## Rationale (why this is worth maintaining)

- **Fault localisation in the cascade.** Once the outer position loop closes
  around this, a misbehaving lander could be the inner loop, the outer loop, or
  their coupling. A passing inner-loop test rules out the inner loop and points
  debugging at the right layer.
- **Catches the bug classes already seen** in development: gain-dict name
  shadowing (wrong overshoot), sign slips (check 3), interface mismatches (test
  won't run).
- **Turns a one-off plot into a permanent contract**, re-verified on every
  refactor, retune, or dynamics change at near-zero cost.
