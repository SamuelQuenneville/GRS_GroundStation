# Control module

Runs the control loop and produces per-tick commands for each vehicle, in
one of three modes. See `docs/ARCHITECTURE.md` for the state-vector layout
and payload-sysId conventions this module defines and everyone else follows.

## `ControlInterface` (`controlInterface.h`/`.cpp`)

Owns the control-loop thread (`m_controlLoop()`, running at `hlcFrequency`
Hz) and, in MPC mode, a `Controller` instance (`MpcController` today). Each tick:

1. Takes the latest telemetry (`updateStates()`'s snapshot).
2. Runs it through `NavigationFrameManager` to get NED-frame states:
   `initializeOffset()` every tick (logs once while there is no GCS origin),
   `toNavigationFrame()` once the frame is initialized.
3. Once the nav frame is initialized, dispatches based on `controlMode`:
   - **MPC**: `ControlStep::tick()` (controller solve, then the NMHE
     sample push and, on its own cadence, the NMHE solve; see below), then
     converts each UAV's raw thrust command through `thrust2rpm()` (see
     `docs/Powertrain.md`), then fires `nmpcDebugCallback` with the
     controller's `DebugInfo`.
   - **MATLAB** — sends states over UDP (`m_sendDataToMatlab`) and blocks
     for a command packet back (`m_receiveDataFromMatlab`) — see the
     top-level `README.md` for the wire struct layout.
   - **ATTITUDE_FILE** — replays a pre-loaded command list
     (`setCommandsList()`) at its own recorded frequency, independent of
     the control loop's `hlcFrequency`.
4. Pushes the resulting commands out via `setCommandCallback()`.

Other public surface: trajectory load/save/generate/preview (delegates to
`Controller`/`TrajectoryGenerator`, see `docs/Trajectory.md`), origin
management (delegates to `NavigationFrameManager`), and
`getPayloadGpsFix()`/`getLiveNavigationStates()` for the dashboard's
trajectory-generator sidebar (see `docs/Dashboard.md`). Control-layer code
never includes Dashboard headers — results flow out through the
`std::function` callbacks set in `GroundControlStation`'s constructor (see
"Event propagation" in `docs/ARCHITECTURE.md`).

## `ControlStep` (`controlStep.h`/`.cpp`)

One MPC-mode tick, shared by `ControlInterface` and `grs_batchsim` (see
`docs/Simulation.md`) so both run identical per-tick logic: hand the
controller the newest finished NMHE estimate (if any), solve the NMPC, push
this tick's sample to the estimator. It never solves the NMHE itself.
Each estimator sample is paired with the control applied over the interval
that ends at that sample, i.e. the previous tick's command. The estimator
only runs in flight (`Controller::inFlight()`): on the launcher and in the
catapult stroke the model does not hold and the estimate would saturate at
its bounds. Samples are pushed and estimates applied only in flight, and the
runner is reset (window empty, estimate zero) at each change, so the first
estimate arrives once the window holds M+1 in-flight samples; until then the
controller uses zero wind and disturbance.
`buildControlStack()` builds the controller and optional estimator from a
YAML profile, the way `ControlInterface::initialize()` does.

## `EstimatorRunner` (`estimatorRunner.h`/`.cpp`)

How the NMHE runs relative to the control loop:

- **`ThreadedEstimatorRunner`** (GCS, SITL, flight): the NMHE has its own
  thread at `nmheFrequency` (wall clock) and owns the `Estimator`. The
  control thread only appends samples to a small queue and picks up the
  latest published estimate, both under short-held mutexes; it never waits
  for an NMHE solve, so a slow NMHE cannot delay a command. An NMHE solve
  longer than its period is counted as an overrun and logged, and the
  schedule restarts from now instead of bursting catch-up solves.
- **`DeferredEstimatorRunner`** (`grs_batchsim`): deterministic emulation of
  the thread. Same cadence, counted in ticks; the solve runs inline so runs
  are reproducible, but its result reaches the controller only after a
  latency (default: the next tick; `--nmhe-latency=<ms>|measured`).

## `Controller` (`controller.h`)

Pure interface `ControlInterface` actually talks to — `solve()`,
`initLaunch()`, trajectory load/save/generate, `getDebugInfo()`,
`getTrajectoryForVehicle()`. Deliberately agnostic to controller family
(NMPC, LMPC, TVLQR, ...); `MpcController` below is the only implementation
today. See `gcs-sitl-integration-plan.md` §3a for why this split exists and
what a second implementation (`TvlqrController`) would look like.

## `MpcController` (`mpcController.h`/`.cpp`)

The `Controller` implementation for the NMPC and the LMPC
(`SolverConfiguration.CONTROLLER: nmpc | lmpc`). `solve(latestStates, time)`
moves the reference window to `time`, fills the solver inputs (initial
state, parameter vector, warm start), runs an `Nlpsol` (see below) and
unpacks the result into per-UAV commands.

The decision vector is `[z0 u0 ... z(N-1) u(N-1) zN]` with the stage state
`z = [x; up]`, `up` the previous control (GRS_Controller
`build_nlp_*_nmpc.m`, `build_nlp_*_lmpc.m`): the control-rate cost is then a
per-stage term, which Fatrop's Hessian represents exactly. `z0` is
`[measured state; U_prev]`, fixed by the initial-condition equalities and
left unbounded; the state bounds apply from the second stage on and `up` is
never bounded (it equals a bounded control). The constructor refuses a
solver generated for another layout.

The last accepted solution is kept as the plan, with the reference sample it
starts at. While it is less than N samples old, it is shifted to the current
sample for the warm start, and a rejected solve applies the plan's control at
that sample instead of the new solution. Otherwise the solve starts cold from
the reference window and a rejected solve applies the reference feedforward
control. `controls.csv` marks rejected solves with the source used.

Launch phases (`m_unpackLatestStates()`):

- **Standby** (before `initLaunch()`): the controller solves every tick, so
  a valid plan exists at release. The UAV position and attitude are
  measured; the velocity is the reference's first one (the launch velocity),
  so the plan is the flight from the launcher. The reference stays at its
  first sample.
- **Launching** (from `initLaunch()`): the reference runs; same state as
  standby until a UAV exceeds `IN_FLIGHT_SPEED` (default 10 m/s). If that
  has not happened `LAUNCH_TIMEOUT` (default 1 s) after the launch
  (misfire), back to standby.
- **In flight**: fully measured state, NMHE on.

`launchReady()` gates `catapultFire` and `initLaunch`: refused without a
checked frame offset for every UAV (below), without a trajectory, before the first solve, with incomplete telemetry, without a
valid plan, or while a UAV is more than `LAUNCH_POS_TOL` (default 3 m) from
its reference start (regenerate the trajectory with the live launch
positions). The three keys are optional `SolverConfiguration` entries.
Commands are sent in standby too (motor running on the launcher if armed).

A control tick that throws (e.g. no trajectory) sends no command and is
logged; the loop keeps running. The NMHE thread logs a failed solve and
keeps the last estimate.

Stale telemetry: in MPC mode, a tick where a UAV's latest position,
velocity or attitude message is older than `GcsConfiguration.telemetryTimeout`
(default 0.3 s, `CommunicationManager::telemetryAge()`) neither solves nor
sends a command, so the autopilot's command timeout applies; logged on each
change. Launch is refused meanwhile.

End of the trajectory: once the reference time passes the last full window,
the controller stops solving and repeats the last applied control every tick
(no AoA feedforward), in the same flight mode. The trajectory ends with a
buffer in which the pilot takes over. Logged once (`TRAJECTORY end`).

No plan left in flight (N rejected solves in a row): the command is the
reference feedforward, open loop, until a solve is accepted again; logged
as an error on each change (`PLAN exhausted`).

Each command also carries the plan's predicted angle of attack
`AOA_FF_STAGE` stages after the applied control (optional
`SolverConfiguration` key, default 1; add the link latency in samples), a
feedforward for the onboard attitude loop. It comes from the alpha rows of
the accepted solution's `g` (linearized for the LMPC) and is 0 while there
is no plan. The tether tension slot is 0 for now.

The LMPC is the same problem with the dynamics and angle-of-attack
constraints linearized: its solver is a QP with the same bounds, constraint
rows and warm start, and its parameter vector is the NMPC's followed by
`P_lin` (each stage's `Ad`, `Bd`, `cd` and alpha rows). A second generated
function computes `P_lin` at every solve from the linearization point, the
wind/disturbance estimate and L0; its time counts in the solve time. Both
are generated by `generate_solvers.m` (GRS_Controller `04_tools/`). The linearization point is the
warm start (the shifted plan, first state measured) when there is a plan,
else the reference window. Linearizing about the plan keeps the affine model
accurate where the solution actually goes, which the reference cannot when
the vehicles are away from it (`run_closed_loop.m`, `lin_point`).

The reference follows the clock: `solve()` gets the time its telemetry was
sampled (steady clock in the GCS, simulated time in `grs_batchsim`), and the
reference window starts at the time elapsed since the first solve after
launch, interpolated linearly between samples. A slow or skipped tick does
not delay the reference, and all UAVs follow the same schedule. The
reference time is followed by the angle-of-attack feedforward [deg] in
`controls.csv`. Also owns:

- **Reference trajectory** — `loadTrajectory()`/`saveTrajectory()` (CSV) and
  `setReferenceTrajectory()` (in-process, from `TrajectoryGenerator`), all
  storing/reading `m_referenceTrajectory` in the solver's own
  `[x0 u0 x1 u1 ... xN uN]` stride.
- **Debug/telemetry readback** — `getDebugInfo()` and
  `getTrajectoryForVehicle()`, both plain structs (defined on `Controller`)
  kept independent of `Dashboard/` (see `docs/ARCHITECTURE.md`).

State-vector and payload-detection conventions (`kUavBlockSize`,
`hasPayload()`) are documented once in `docs/ARCHITECTURE.md` rather than
repeated here.

## `NmheEstimator` (`nmheEstimator.h`/`.cpp`)

The `Estimator` implementation (GRS_Controller `build_nmhe_*.m`). Stage state
`[x; wind; d]` over the last M+1 samples, wind and d constant (identity
dynamics), measurement fit plus an arrival cost on the previous estimate.
Wind and d are bounded on the first stage only (`WIND_BOUND`, `D_BOUND`),
which the identity dynamics carry to the whole window. Each solve warm starts
from the previous valid solution moved back by the samples added since
(`shift_nmhe.m`); the new stages take the measured state and the last
wind/d. Without a valid previous solution it starts cold (measured states,
prior wind/d). An invalid solve keeps the previous estimate.

Disturbance layouts: one UAV `[dFx dFy dFz b_roll b_pitch]`; two UAVs
`[dFa1 dCL1 b_roll1 b_pitch1 dFa2 dCL2 b_roll2 b_pitch2 dFz_pay]` (force
along the airspeed, lift coefficient offset, attitude-command biases per
UAV, vertical force on the payload).

## `GeneratedFunction` / `Nlpsol` (`generatedFunction.h`, `nlpsol.h`)

`GeneratedFunction` is one CasADi-generated function from `CasadiSolver/`
(its memory slot and workspaces), picked by id (NMPC, LMPC, LMPC
linearization, NMHE) and `NUM_UAVS`. Every generated header is included only
in `generatedFunction.cpp`; adding one means one `GENERATED_API(...)` entry
in its table. `Nlpsol` adds the nlpsol buffers (8 in, 6 out, nlpsol order)
and the solution check. The parameter and bound layouts belong to the
problem, not the solver: `MpcController` and `NmheEstimator` pack them.

The generated nlpsol functions return 0 even when Fatrop does not converge.
CMake renames their call to `fatrop_ocp_c_solve()` so it goes through
`fatropStatus.cpp`, which records Fatrop's return code (0 converged, 1
iteration limit) and iteration count; `Nlpsol::solve()` returns them. They
reach `DebugInfo::lastFatrop`, the dashboard and `controls.csv`. Fatrop takes no dual initial guess, so `lamX0`/`lamG0` stay zero.

## `ControlDispatcher` (`controlDispatcher.h`/`.cpp`)

Small queue/thread that decouples `ControlInterface` (producer of commands)
from `CommunicationManager` (consumer, sends them to the vehicles) and vice
versa for telemetry — `attachCommunicationManager()`/
`attachControllerInput()` wire the two `std::function` callbacks together.
Runs its own dispatch thread (`m_dispatchLoop()`) reading off
`m_commandQueue`, which holds only the latest command: an unsent one is
replaced, never sent late (drops are logged).

## `NavigationFrameManager` (`navigationFrameManager.h`/`.cpp`)

Converts WGS84 GPS states into a local NED frame anchored at an operator-set
origin, using `Geo::GeodeticConverter` (see `docs/Geo.md`). Two related but
distinct states:

- `hasOrigin()` — an origin has been set (`setOrigin()`), so there's
  something to show on the setup UI.
- `isInitialized()` — additionally, `initializeOffset()` has computed a
  per-UAV frame offset. Nothing is converted until this is true.

A vehicle's position in the GCS frame is its `LOCAL_POSITION_NED` plus its
offset, the position of its EKF origin in the GCS frame. The EKF origin
comes from `GPS_GLOBAL_ORIGIN` (`CommunicationManager` requests it every 2 s
until it arrives, then every 10 s, and passes it on through
`setEkfOrigin()`), so the offset is exact and does not depend on two
messages sampled at different times. The same holds in SITL. The tangent
planes of the two origins differ by less than 0.2 m per km of separation.

`initializeOffset()` is called unconditionally every control-loop tick; it's
incremental: it computes the offset of a sysId once its EKF origin is known
(refused above 5 km: wrong GCS or EKF origin, logged once), and checks every
offset while the vehicle is still (< 1 m/s): its `GLOBAL_POSITION_INT`
must equal the local position plus the offset within 1 m, otherwise an
error is logged. `frameReady()` requires a checked offset for every UAV and
gates `catapultFire`/`initLaunch` (see `launchReady()`). `setOrigin()` clears
every offset, and a changed EKF origin clears that vehicle's offset; both
are recomputed on the next tick. All public methods lock `m_mutex`, since
`setOrigin()` can be called from the console or dashboard thread and
`setEkfOrigin()` from a MAVSDK callback while the control loop is
concurrently reading/writing state every tick.
