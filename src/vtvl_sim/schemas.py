"""Pydantic models for validating scenario JSON files before they reach the solver.

These validate scenario *setup* once per run (initial state, targets, gains,
solver options) — not the per-step ODE state passed through solve_ivp, which
stays a plain list/array for speed.
"""

from typing import Literal

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    NonNegativeFloat,
    PositiveFloat,
    model_validator,
)

from vtvl_sim.controllers import CONTROLLER_REGISTRY


class LanderState(BaseModel):
    model_config = ConfigDict(extra='forbid')

    x: float
    z: NonNegativeFloat
    xdot: float
    zdot: float
    theta: float
    thetadot: float
    m: PositiveFloat

    def to_list(self) -> list[float]:
        return [self.x, self.z, self.xdot, self.zdot, self.theta, self.thetadot, self.m]


class Phase(BaseModel):
    model_config = ConfigDict(extra='forbid')

    x_target: float
    z_target: NonNegativeFloat
    t_end: PositiveFloat
    # Pitch reference [deg] for the attitude-hold demo controller; ignored by the
    # mission controllers (Cascaded PD, Altitude PID). Defaulted so scenario files
    # written before it existed still validate under extra='forbid'.
    theta_target_deg: float = 0.0


class ParamsSchema(BaseModel):
    model_config = ConfigDict(extra='forbid')

    m_dry: PositiveFloat
    I: PositiveFloat
    L: PositiveFloat
    g: PositiveFloat
    T_max: PositiveFloat
    T_min: NonNegativeFloat
    isp: PositiveFloat
    delta_max_deg: float = Field(gt=0, le=90)
    tilt_limit_deg: float = Field(gt=0, le=90)

    @model_validator(mode='after')
    def check_thrust_bounds(self):
        if self.T_min >= self.T_max:
            raise ValueError(f'T_min ({self.T_min}) must be < T_max ({self.T_max})')
        return self


class ScenarioSetup(BaseModel):
    model_config = ConfigDict(extra='forbid')

    params: ParamsSchema
    controller_name: str
    gains: dict[str, float]
    phases: list[Phase] = Field(min_length=1)
    initial_state: LanderState
    landing_tolerance: PositiveFloat

    @model_validator(mode='after')
    def check_controller_and_gains(self):
        if self.controller_name not in CONTROLLER_REGISTRY:
            raise ValueError(
                f'unknown controller_name {self.controller_name!r}, '
                f'available: {list(CONTROLLER_REGISTRY)}'
            )
        required = set(CONTROLLER_REGISTRY[self.controller_name]['gain_fields'])
        missing = required - self.gains.keys()
        if missing:
            raise ValueError(
                f'gains missing required fields for {self.controller_name!r}: {sorted(missing)}'
            )
        return self
    @model_validator(mode='after')
    def check_initial_mass(self):
        if self.initial_state.m <= self.params.m_dry:
            raise ValueError(
                f'initial mass {self.initial_state.m} must be greater than dry mass {self.params.m_dry}'
            )
        return self


class SolverSetup(BaseModel):
    model_config = ConfigDict(extra='forbid')

    max_step: PositiveFloat
    method: Literal['RK45', 'RK23', 'DOP853', 'Radau', 'BDF', 'LSODA']


class Outputs(BaseModel):
    model_config = ConfigDict(extra='forbid')

    trajectory: Literal[1, 0]
    state: Literal[1, 0]
    animation: Literal[1, 0]
    report: Literal[1, 0]
    csv: Literal[1, 0]
    # Defaulted (unlike its siblings) so scenario files written before the engine
    # plot existed still validate — extra='forbid' would otherwise reject them.
    engine: Literal[1, 0] = 1
    # Same reasoning: defaulted so scenario files predating the propellant plot
    # still validate.
    propellant: Literal[1, 0] = 1