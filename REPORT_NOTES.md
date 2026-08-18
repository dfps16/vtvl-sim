# Report Notes

Running log of findings worth writing up in the final technical report. Appended to as
they come up during implementation — newest section at the bottom of each theme.

Distinguish two kinds of entry:
- **[verified]** — checked numerically against the code in this repo during implementation.
- **[from plan]** — asserted in `lqr_plan.md` / `lqr_implementation_notes.md`, carried here
  because it belongs in the report, but not independently re-checked yet.

---

## 1. Modelling and linearisation

### 1.1 The trim point is exact for any constant velocity [from plan]

Because the model has no drag or any other velocity-dependent force, the trim
`θ = δ = 0`, `T = mg` holds for *any* constant `ẋ, ż` — not just zero velocity. The linear
model is therefore not restricted to "near-hover"; its only approximation is the
small-angle assumption on `θ`. Worth stating explicitly, since "linearised about hover"
usually implies a much tighter validity region than actually applies here.

### 1.2 `A` is nilpotent with index 4 [verified]

`A⁴ = 0` numerically, and all six eigenvalues sit at the origin. This is the matrix-level
statement of `det(sI − A) = s⁶`: six integrators, no natural restoring force anywhere in
the plant. Two consequences:

- The open-loop plant is not merely marginally stable — a gimbal disturbance grows as
  `t⁴`. Feedback is structurally necessary, not a performance improvement.
- The controllability matrix is really only `[B, AB, A²B, A³B]` padded with two zero
  blocks. Useful diagnostically: a rank deficiency of 1 points at a broken integrator
  chain (a missing kinematic `1`), not at a sign error.

### 1.3 Controllability confirmed numerically [verified]

`rank(ctrb(A, B)) = 6` at nominal `PARAMS`, matching the closed-form argument
(notes §1.9: z-block determinant `−1/m²`, x-θ block determinant `g²c⁴`, both nonzero for
any physically realisable vehicle). The closed-form result is the stronger claim and
should lead in the report; the numerical check is the guard against sign errors, which it
would not otherwise catch.

### 1.4 The corrected lateral transfer function [from plan]

`lqr_implementation_notes.md` §1.8 gives `X(s)/dδ(s) = g²c/s⁴`. This is wrong — it drops
the direct gimbal→lateral feedthrough that the same document's `B` matrix contains
(`B[2,1] = +g`). The correct channel is

```
ẍ = −g·θ + g·dδ ,   θ̈ = −c·dδ ,   c ≡ mgL/I = 4.905
X(s)/dδ(s) = g/s² + gc/s⁴ = g(s² + c)/s⁴
```

so there is a pair of transmission zeros at `s = ±j√c = ±j2.215`, not a pure quadruple
integrator. Both paths push `x` the same direction (positive gimbal shoves `+x` directly
*and* pitches nose-negative, which shoves `+x` again), so the channel is minimum-phase.

**Report angle:** the cascade's outer loop assumes `ẍ ≈ −gθ` (`controllers.py:175-176`)
and so ignores the direct term entirely; LQR uses it. That is a *structural* advantage of
the state-space design, not a tuning difference — a good concrete answer to "why not just
tune the cascade harder."

---

## 2. LQR weighting

### 2.1 What Bryson's rule actually buys [verified — reasoning]

The cost `J = ∫ eᵀQe + uᵀRu dt` is dimensionally meaningless as written: it adds metres²
to radians² to newtons². Weighting by `qᵢ = 1/dᵢ²` for a declared acceptable deviation
`dᵢ` turns each term into `(eᵢ/dᵢ)²` — dimensionless, and exactly 1 when that variable
sits at its acceptable deviation. The cost then reads as "how many acceptable-deviations
am I off, squared, summed." That is what makes a 20 m lateral error commensurate with a
0.17 rad tilt.

Frame it as a *unit-normalisation device* first and a tuning heuristic second — the plan's
`z_dev` sweep (§4.2 below) shows it is a poor heuristic but the normalisation is sound
regardless.

### 2.2 Only the ratios matter [verified — reasoning]

If `P` solves the CARE for `(Q, R)`, then `αP` solves it for `(αQ, αR)`, and
`K = R⁻¹BᵀP` is unchanged. The 8 Bryson numbers are therefore 7 degrees of freedom — the
design sets relative priorities, never absolute ones. "Tighten `x_dev`" and "loosen
everything else" are the same move.

### 2.3 A closed-form gain: `k_z = T_dev / z_dev` [verified]

The z-channel is a pure double integrator (`z̈ = dT/m`), and its LQR solution has an
unusually clean answer. For `ẋ₁ = x₂`, `ẋ₂ = b·u` with cost `q₁x₁² + q₂x₂² + ru²`, solving
the 2×2 CARE gives `p₂ = √(q₁r)/b`, hence

```
k₁ = (b/r)·p₂ = √(q₁/r)     — independent of b, so independent of mass
```

Substituting Bryson's `q₁ = 1/z_dev²`, `r = 1/T_dev²`:

```
k_z = T_dev / z_dev = 750/10 = 75.0     (matches computed K[0,1] = 75.0000 exactly)
k_ż = √((2p₂ + q₂)/r) = 413.07          (matches computed K[0,3] = 413.0678)
```

**Report angle:** the thrust gain on altitude error is *literally the ratio of the two
acceptable deviations you declared* — newtons of thrust deviation per metre of altitude
error. It is a rare case where Bryson's rule has a direct, defensible physical reading
rather than being a black-box heuristic, and the mass-independence is a genuinely
non-obvious result worth a sentence.

### 2.4 The closed form generalises — and it applies exactly to the cyclic coordinates [verified]

§2.3 derived `k_z = T_dev/z_dev` for the double integrator. The same form holds in the
lateral channel, which is a *four*-integrator chain with a direct feedthrough term:

```
k_x = delta_dev / x_dev = 0.2094/20 = 0.01047     (computed K[1,0] = 0.01047000)
```

Checked against a second, arbitrary weight set (`x_dev=7.3, delta_dev=0.05, z_dev=41,
T_dev=310`): predicted `0.00684932` and `7.56097561`, computed identically. Not a
coincidence of the tuned numbers.

**Why, in general.** For a single-input channel, take the `(1,1)` entry of the CARE
`AᵀP + PA − PBR⁻¹BᵀP + Q = 0`. The term `(AᵀP + PA)₁₁` is a sum over column 1 of `A`. If
the first state appears in *no* equation of the dynamics — i.e. column 1 of `A` is
identically zero — that term vanishes and the equation collapses to

```
q₁ = (1/r)(Bᵀ P)₁²   ⇒   (BᵀP)₁ = √(q₁ r)   ⇒   k₁ = (1/r)(BᵀP)₁ = √(q₁/r)
```

independent of everything else in the plant — including the mass, the moment arm, and the
length of the integrator chain. Substituting Bryson's `q₁ = 1/d₁²`, `r = 1/d_u²` gives
`k₁ = d_u/d₁`.

**The condition "column of `A` is identically zero" is exactly the definition of a cyclic
coordinate** from the trim analysis (notes §1.2). So:

> The states identified as cyclic during trim are precisely the states whose LQR feedback
> gains have a closed form — and that form is the ratio of the two acceptable deviations
> declared for them.

Here that is `x` and `z`, the two position states, and it is why *both* `k_x` and `k_z`
come out as clean ratios while the velocity and attitude gains do not.

**Report angle:** this is the strongest single result to come out of the implementation. It
ties together three otherwise separate threads — the trim/cyclic-coordinate analysis (§1),
the Bryson normalisation argument (§2.1), and the numerical `K` — and it turns Bryson's
rule from a black-box heuristic into something with a directly defensible physical reading
for the position channels: *newtons of thrust deviation per metre of altitude error*, and
*radians of gimbal per metre of lateral error*, exactly as declared.

### 2.5 The rule is visible in the very first control command [verified]

At `t = 0` in the reference scenario the lateral error is exactly 20 m and `x_dev = 20.0`,
so by §2.4 the commanded gimbal is exactly `delta_dev = 0.2094 rad = 11.9977°`. The limit
is `delta_max = radians(12) = 0.20944 rad`. The command starts within **0.003°** of
saturation and decreases monotonically thereafter.

This is Bryson's normalisation made observable: at one acceptable-deviation of position
error, the controller commands exactly one acceptable-deviation of control. It also
explains the plan's "gimbal saturated: 0%" — that is not a lucky tuning outcome but a
direct consequence of setting `x_dev` to the full initial offset and `delta_dev` to the
actuator limit.

Contrast the vertical channel at the same instant: `T_cmd = −5538 N`, a *negative* thrust
demand (LQR asking to fall faster), clipped to `T_min = 1000 N`. The two channels behave
completely differently on step one — the lateral one sits exactly at its declared budget,
the vertical one is far outside any physical range — because `z_dev = 10` against a 100 m
error is ten acceptable-deviations, while the lateral error is one.

### 2.6 The weight spread explains the slow descent [verified]

At the tuned defaults:

| | dev | weight |
|---|---|---|
| `x` | 20.0 | 0.0025 |
| `z` | 10.0 | 0.01 |
| `ẋ` | 10.0 | 0.01 |
| `ż` | 2.0 | 0.25 |
| `θ` | 0.1745 | 32.84 |
| `θ̇` | 0.3 | 11.11 |
| `T` | 750.0 | 1.778e-6 |
| `δ` | 0.2094 | 22.81 |

`R_T ≈ 1.8e-6` against `Q_θ ≈ 33` — seven orders of magnitude. Thrust is nearly free in
this cost function, which is *why* LQR selects a slow, overdamped, low-effort descent
(dominant pole `−0.201`, ≈5 s time constant, driving the whole ~25 s trajectory). This is
not a defect to be tuned away; it is the direct consequence of declaring 750 N of thrust
deviation acceptable while declaring 0.17 rad of tilt acceptable.

### 2.7 Asymmetric throttle authority: `T_dev = 750` overstates it [from plan]

`(T_max − T_min)/2 = 750`, but the range about hover is asymmetric:

```
up:    T_max − mg = 2500 − 1962 = 538 N
down:  mg − T_min = 1962 − 1000 = 962 N
```

The binding constraint is 538 N. `T_dev = 750` tells the design it has ~40% more *upward*
authority than it has. Trajectory impact is small (the `z_dev`/`zdot_dev` ratio dominates)
but `T_dev = 538` is the honest number.

**Note the interaction with §2.3:** since `k_z = T_dev/z_dev` exactly, switching to 538
changes the gain to 53.8 — so this choice is not cosmetic at the gain level even where it
is nearly invisible at the trajectory level. Worth stating which was used and why.

---

## 3. Structure and decoupling

### 3.1 Block-sparse `K`, exactly [verified]

```
K = [[ 0      75.0     0     413.0678   0        0     ]   ← thrust: z, ż only
     [ 0.0105  0       0.0625   0      −1.6225  −0.9517]]  ← gimbal: x, ẋ, θ, θ̇ only
```

The structural zeros are exact to machine precision (`atol=1e-9`), not numerical noise.
This is theory confirmed empirically: block-diagonal `A`, `B` (from trimming at `θ = 0`)
plus diagonal `Q`, `R` ⇒ the Riccati solution inherits the block split.

Practical payoff: the two channels tune completely independently — the plan reports that
sweeping the lateral weights moved touchdown time and vertical velocity by <0.1%. Tune
vertical first, then lateral, never both at once. That is a methodology result, not just
an observation.

Caveat worth stating: the split is a consequence of the **zero-tilt trim choice**. It
would not survive linearisation about a nonzero steady tilt, which is what a
trajectory-tracking (LTV) design would require.

### 3.2 Closed-loop poles [verified]

```
z-channel:    −0.2012, −1.8642                (overdamped)
x-θ channel:  −2.5653, −2.2713, −0.222 ± 0.1928j
```

All in the open left half-plane; matches the plan's predicted values. The dominant
`−0.201` is the whole explanation for the ~25 s descent.

---

## 4. Methodology notes (process, worth a short section)

### 4.1 A verification test that does not verify what it claims [verified]

`lqr_plan.md` §5 proposes `test_lqr_gain_structure` (asserting the structural zeros of
`K`) as the guard against a state-ordering transposition in `bryson_weights`. **It does
not work.** Permuting the diagonal entries of a diagonal `Q` leaves it diagonal, so the
block split of `A`/`B` still yields a block-sparse `K` and the test passes clean.

Worse, at the tuned defaults the bug is *invisible* in the weights themselves:

```
correct ordering    [x_dev, z_dev, xdot_dev, zdot_dev] = [20, 10, 10, 2]
per-channel grouping[x_dev, xdot_dev, z_dev, zdot_dev] = [20, 10, 10, 2]
```

identical, because `z_dev` and `xdot_dev` both happen to be 10. The controller would look
correct until someone edited a gain in a scenario file.

What actually catches it is the closed-form check of §2.3: `K[0,1] = T_dev/z_dev` confirms
the weight landed in the z-slot specifically, independent of the coincidence.

**Report angle:** a clean, small example of a test that passes for the wrong reason, and
of preferring a check against an independently-derived analytic value over a check against
a structural property the system would exhibit anyway.

### 4.2 Bryson's rule is a starting point, not a design [from plan]

The `z_dev` sweep in `lqr_plan.md` §3 is the evidence: the notes' original `z_dev = 2.0`
(chosen as a landing *accuracy*) crashes the vehicle at −15.0 m/s with thrust saturated 97%
of the run, because it demands a 100 m error be zeroed as though 2 m were tolerable.
`z_dev = 10` (the scale of the *manoeuvre*) lands at −0.20 m/s.

The transferable lesson: acceptable-deviation numbers must reflect the **scale of the
manoeuvre**, not the terminal tolerance.

Counter-intuitive companion result in the lateral channel: *tightening* `x_dev` makes the
response more aggressive and *less* accurate (17.3° peak tilt, −0.22 m error at
`x_dev = 5` vs 7.0° and 0.14 m at `x_dev = 20`), because it over-drives the gimbal into
saturation early and wastes authority during the transient.

---

## 5. Stated limitations (collect here, write up as one section)

- **No state constraint on tilt.** `sim.py:13-14` clips only `T` and `δ`. The cascade
  self-limits by clipping `θ_ref` to `tilt_limit`; unconstrained LQR has no mechanism to
  respect a state constraint at all. This is inherent to the method, not an implementation
  gap — and it is the motivation for constrained/convex guidance later. [from plan]
- **Saturation is applied after the fact.** LQR is designed ignoring `[T_min, T_max]` and
  `±δ_max`, then `sim.py` clips the output. During the ~11% of the run where thrust
  saturates, the realised control is *not* the optimal one. [from plan]
- **Linear-model optimality only.** Gains are optimal for the linearised plant; nonlinear
  tracking error grows as `O(θ²)` (cos terms) and `O(θ³)` (sin terms), worst near the
  actuator limits. At `δ_max = 12°`, the `1 − cos δ ≈ 2.2%` term is dropped entirely from
  `z̈` by the linear model. [from plan]
- **The `K` cache assumes constant mass.** Valid only while `params['m']` is fixed;
  invalid the moment mass depletion enters the state vector, which would require per-step
  Riccati re-solves or gain-scheduling on `m`. [from plan]

---

## 6. Head-to-head comparison [verified]

Reproduced by running both scenarios end-to-end through `sim_run` on identical geometry
(100 m drop, 20 m lateral offset, 1 m landing tolerance): `test_scenarios/default.json`
(Cascaded PD) and `test_scenarios/lqr1.json` (LQR, tuned defaults). Matches `lqr_plan.md`
§4's predicted table to within rounding.

| | Cascaded PD | LQR (tuned) |
|---|---|---|
| Touchdown time | 17.57 s | 24.75 s |
| Touchdown ż | −0.347 m/s | **−0.201 m/s** |
| Lateral error | −0.351 m | **+0.142 m** |
| Peak \|θ\| | 10.13° | **7.04°** |
| Thrust saturated | 13.8% | 11.2% |
| Gimbal saturated | 3.4% | **0.0%** |

(The plan's table left "Gimbal saturated" as "—" for the cascade, presumably unmeasured
rather than inapplicable — the cascade's inner loop clips `δ` to `±δ_max` the same as LQR,
`controllers.py:303`. Measured here at 3.4%, so LQR's "never touches the gimbal limit" is a
real, not merely unreported, difference.)

Headline: **the cascade buys speed with actuator aggression; LQR spends time to stay in the
linear regime.** Both are design choices — encoded in `Q`/`R` for LQR, in `ζ`/`ωₙ` for the
cascade — but the LQR one is explicit and re-tunable through a single scalar ratio.