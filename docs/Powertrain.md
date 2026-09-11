# Powertrain module

Free functions (`powertrain.h`/`.cpp`) converting between commanded thrust
(Newtons) and motor RPM for one specific propeller (APC 16x8E), using a
thrust model fit from wind-tunnel testing (the `x`/`y`/`z` coefficients
repeated across every function).

- **`thrust2rpm(airspeed, thrustTarget)`** — the one actually called from
  the control loop (`ControlInterface::m_controlLoop()`, MPC mode) to turn
  the solver's thrust command into a normalized `[0,1]` RPM command.
  Solves the (nonlinear in RPM) thrust model with Newton-Raphson
  (`evalThrustModel()` gives both the function value and its derivative),
  capped at `MAX_ITER_RPM` iterations, then saturates to `[0, 9000]` RPM
  before normalizing.
- **`rpm2thrust(airspeed, rpmTarget)`** — the inverse (closed-form, no
  iteration needed).
- **`maxThrust(airspeed)`** — the model's thrust ceiling at a given
  airspeed (RPM fixed at 9000).

All three re-derive `rho`/`D`/`x`/`y`/`z` locally rather than sharing them —
flagged as a simplification candidate (see the separate simplification
list) since they're the same physical constants in every function.
