# Powertrain module

Free functions (`powertrain.h`/`.cpp`) converting between commanded thrust
(Newtons) and motor RPM for one specific propeller (APC 16x8E), using a
thrust model fit from wind-tunnel testing. With `n` in RPM and
`phi = 60*airspeed/D`, the model is quadratic in `n`:
`thrust = psi * (z*n^2 - y*phi*n - x*phi^2)`, `psi = rho*D^4/3600`.

- **`thrust2rpm(airspeed, thrustTarget)`**: called from the control loop
  (`ControlInterface::m_controlLoop()`, MPC mode) to turn the solver's
  thrust command into a normalized `[0,1]` RPM command: the positive root
  of the quadratic, saturated to `[0, 9000]` RPM. Always finite: a
  non-finite target or one of 2 N or less gives 0 (motor off), a
  non-finite or negative airspeed is taken as 0.
- **`rpm2thrust(airspeed, rpmTarget)`**: the inverse.
- **`maxThrust(airspeed)`**: the model's thrust at 9000 RPM.
